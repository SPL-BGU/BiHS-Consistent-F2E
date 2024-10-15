#!/bin/bash

# Usage function to display help message
usage() {
    echo "Usage: $0 [options]"
    echo
    echo "Options:"
    echo "  -s, --stp        Run all algorithms for the STP (Sliding Tile Puzzle) domain."
    echo "  -t, --toh        Run all algorithms for the TOH (Tower of Hanoi) domain."
    echo "  -p, --pancake    Run all algorithms for the Pancake Sorting domain."
    echo "  -r, --road       Run all algorithms for the Road Navigation domain."
    echo "  -d, --dao        Run all algorithms for the DAO domain."
    echo "  -h, --help       Show this help message and exit."
    echo
    echo "Example:"
    echo "  $0 --stp --road"
    echo "    Runs all algorithms for both the STP and Road domains."
    exit 1
}


# Change the working directory to the directory where the parent of script
cd "$(dirname "$0")" || exit 1
cd .. || exit 1

mkdir -p data
mkdir -p data/conference
mkdir -p data/conference/pancake
mkdir -p data/conference/road
mkdir -p data/conference/dao
mkdir -p data/conference/stp
mkdir -p data/conference/toh

# Parse command-line arguments
stp_flag=false
toh_flag=false
pancake_flag=false
road_flag=false
dao_flag=false

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -s|--stp) stp_flag=true ;;
        -t|--toh) toh_flag=true ;;
        -p|--pancake) pancake_flag=true ;;
        -r|--road) road_flag=true ;;
        -d|--dao) dao_flag=true ;;
        -h|--help) usage ;;
        *) echo "Unknown option: $1"; usage ;;
    esac
    shift
done

release_algs=("BAE*-o-a" "BAE*-o-p" "A*" "NBS" "DVCBS" "GMX" "RA*" "NGMX"
              "BAE*-fd-a" "BAE*-fd-p"
              "BAE*-rfrd-a" "BAE*-rfrd-p"
              "BAE*-df-a" "BAE*-df-p"
              "BAE*-rdrf-a" "BAE*-rdrf-p"
              "BAE*-g-a" "BAE*-g-p"
              "BAE*-b-a" "BAE*-b-p"
              "BAE*-frdrfd-a" "BAE*-frdrfd-p"
              "BAE*-gfgd-a" "BAE*-gfgd-p"
              "BAE*-rfdfrd-a" "BAE*-rfdfrd-p"
              "BAE*-rfrdx2-a" "BAE*-rfrdx2-p"
              "BAE*-grfgrd-a" "BAE*-grfgrd-p"
              "BAE*-gdgf-a" "BAE*-gdgf-p"
              "BAE*-grdgrf-a" "BAE*-grdgrf-p"
              "BAE*-gb-a" "BAE*-gb-p"
              "BAE*-gfrdgrfd-a" "BAE*-gfrdgrfd-p"
              "BAE*-grfdgfrd-a" "BAE*-grfdgfrd-p"
              "BAE*-grfrd-a" "BAE*-grfrd-p")

debug_algs=("DBBS-a" "DBBS-p" "DBBS-a-MinTot" "DBBS-o-MinTot"
            "DBBSLB-a" "DBBSLB-p" "DBBSLB-a-MinTot" "DBBSLB-o-MinTot")

cd conference/bin/release || exit 1
for alg in "${release_algs[@]}"; do
  if $stp_flag; then
    ./bidirectional -d stp -h md -i 0 -n 100 -a "$alg" | tee "../../../data/conference/stp/stp_${alg//\*/}.out"
  fi

  if $toh_flag; then
    for i in 2 4 6; do
      ./bidirectional -d toh -h "$i" -i 0 -n 50 -a "$alg" | tee "../../../data/conference/toh/toh_${i}_${alg//\*/}.out"
    done
  fi

  if $pancake_flag; then
    for i in {0..5}; do
      ./bidirectional -d pancake -h "$i" -i 0 -n 50 -a "$alg" | tee "../../../data/conference/pancake/pancake_${i}_${alg//\*/}.out"
    done
  fi

  if $road_flag; then
    ./bidirectional -d road -h eucspeed -i 0 -n 100 -a "$alg" | tee "../../../data/conference/road/road_${alg//\*/}.out"
  fi

  if $dao_flag; then
    ./bidirectional -d dao -h all -a "$alg" | tee "../../../data/conference/dao/dao_${alg//\*/}.out"
  fi
done

cd ../debug || exit 1
for alg in "${debug_algs[@]}"; do
  if $stp_flag; then
    ./bidirectional -d stp -h md -i 0 -n 100 -a "$alg" | tee "../../../data/conference/stp/stp_${alg//\*/}.out"
  fi

  if $toh_flag; then
    for i in 2 4 6; do
      ./bidirectional -d toh -h "$i" -i 0 -n 50 -a "$alg" | tee "../../../data/conference/toh/toh_${i}_${alg//\*/}.out"
    done
  fi

  if $pancake_flag; then
    for i in {0..5}; do
      ./bidirectional -d pancake -h "$i" -i 0 -n 50 -a "$alg" | tee "../../../data/conference/pancake/pancake_${i}_${alg//\*/}.out"
    done
  fi

  if $dao_flag; then
    ./bidirectional -d dao -h all -a "$alg" | tee "../../../data/conference/dao/dao_${alg//\*/}.out"
  fi
done
