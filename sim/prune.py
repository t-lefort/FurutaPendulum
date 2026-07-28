"""
Élagage APRÈS entraînement, pour faire tenir une politique en DMAMEM.

    python -m sim.prune runs/q_long_s1.bin

POURQUOI APRÈS. Comprimer AVANT l'entraînement (hachage plus agressif,
résolution plus faible) a été mesuré comme systématiquement destructeur : les
collisions font que le swing-up et l'équilibre s'écrivent dans les mêmes poids et
se détruisent mutuellement. Mesures, tenue en glouton :
    dense 100x30 (4747 ko) : swing-up 100 %, tenue 12,95 s
    2^15 + warp  ( 448 ko) : swing-up 100 %, tenue  1,01 s
    2^13 + warp  ( 112 ko) : swing-up   0 %, tenue  0,00 s
Élaguer après, en revanche, comprime une solution DÉJÀ trouvée : la collision ne
peut plus casser un apprentissage terminé.

CE QU'ON PEUT JETER SANS RIEN CHANGER À LA POLITIQUE. En inférence seule
importe l'action argmax, donc un trait dont les 7 poids sont quasi identiques
n'apporte qu'un décalage constant : il ne change aucun choix. On mesure donc
l'ÉCART (max - min) sur les actions de chaque trait, et on jette ceux sous un
seuil. /!\\ Ça ne vaut que pour l'inférence : la valeur absolue, elle, sert au
bootstrapping, donc une table élaguée ne se ré-entraîne pas telle quelle.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from array import array
from pathlib import Path

from .fw_config import load
from .runner import Runner

_HDR = "<IHHHH"
_MAGIC = 0x54494C31

# Réglages sous lesquels la table dense a été entraînée. Doivent correspondre,
# sinon la géométrie des traits ne veut plus rien dire.
BASE = dict(TC_SPLIT="0", QL_SARSA="1", QL_K_ENERGY="8",
            TC_TILINGS="8", TC_N_ALPHA="100",
            TC_N_ADOT="30", TC_N_TDOT="6", QL_U_MIN="0.06", RL_DIVIDER="5",
            QL_GAMMA="0.999", QL_U_TAU="0.002", DUTY_SLEW_PER_S="80",
            QL_LAMBDA="0.92", QL_LR="0.01", QL_THETA_TURNS="6", QL_K_BAL="10")


def read_weights(path: str):
    with open(path, "rb") as f:
        magic, ver, T, per, na = struct.unpack(_HDR, f.read(12))
        if magic != _MAGIC:
            raise SystemExit(f"{path} : ce n'est pas une table tile coding")
        W = array("f")
        W.frombytes(f.read())
    return W, T, per, na


def spreads(W, na: int):
    """Écart max-min sur les actions, trait par trait."""
    out = array("f", bytes(4 * (len(W) // na)))
    for i in range(0, len(W), na):
        lo = hi = W[i]
        for a in range(1, na):
            v = W[i + a]
            if v < lo:
                lo = v
            elif v > hi:
                hi = v
        out[i // na] = hi - lo
    return out


def prune(W, na: int, sp, thr: float) -> tuple[array, int]:
    """Met à zéro les traits dont l'écart est sous le seuil. Retourne (poids, gardés)."""
    Wp = array("f", W)
    kept = 0
    for f in range(len(sp)):
        if sp[f] < thr:
            base = f * na
            for a in range(na):
                Wp[base + a] = 0.0
        elif sp[f] > 0.0:
            kept += 1
    return Wp, kept


def evaluate(cfg, rig, W, episodes: int = 12):
    r = Runner(cfg, rig, mode="ql_greedy", seed=4242)
    r.rig.curriculum_frac = 0.0
    r.agent.W = W
    r.agent.greedy = True
    r.agent.epsilon = 0.0
    s = r.run_episodes(episodes)
    holds = sorted((e.t_balance for e in s.episodes), reverse=True)
    return s.swingup_rate(episodes), holds[len(holds) // 2], holds[-1]


def main(argv=None) -> None:
    p = argparse.ArgumentParser(prog="python -m sim.prune",
                                description="Élagage post-entraînement.")
    p.add_argument("table")
    p.add_argument("--out", metavar="FICHIER.bin",
                   help="écrit la table élaguée au meilleur seuil retenu")
    p.add_argument("--seuils", default="0,0.01,0.05,0.2,0.5,1,2,5",
                   help="seuils d'écart à tester")
    args = p.parse_args(argv)

    W, T, per, na = read_weights(args.table)
    cfg, rig = load(dict(BASE), {})
    if len(W) != T * per * na:
        raise SystemExit("taille de table incohérente avec l'entête")

    sp = spreads(W, na)
    nz = sum(1 for v in sp if v > 0.0)
    print(f"{args.table} : {len(sp)} traits, {nz} visités "
          f"({100 * nz / len(sp):.1f} %), {len(W) * 4 / 1024:.0f} ko dense\n")
    print(f"{'seuil':>7} {'gardés':>8} {'%':>6} {'int16':>8}  swing-up   tenue med.   tenue min")
    print("-" * 70)

    best = None
    for raw in args.seuils.split(","):
        thr = float(raw)
        Wp, kept = prune(W, na, sp, thr)
        su, med, mn = evaluate(cfg, rig, Wp)
        ko = kept * na * 2 / 1024
        flag = ""
        if su >= 0.99 and med >= 8.0:
            flag = "  <= tient"
            if best is None or kept < best[1]:
                best = (thr, kept, Wp)
        print(f"{thr:7.2f} {kept:8d} {100 * kept / len(sp):5.1f}% "
              f"{ko:7.0f}k {100 * su:8.0f}% {med:11.2f}s {mn:11.2f}s{flag}")

    if best and args.out:
        thr, kept, Wp = best
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        with open(args.out, "wb") as f:
            f.write(struct.pack(_HDR, _MAGIC, 1, T, per, na))
            f.write(Wp.tobytes())
        print(f"\nseuil retenu {thr} ({kept} traits, "
              f"{kept * na * 2 / 1024:.0f} ko en int16) -> {args.out}")
    elif args.out:
        print("\naucun seuil ne préserve la politique : rien écrit.")


if __name__ == "__main__":
    main()
