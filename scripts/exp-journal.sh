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
    echo "  -l, --linear     Run the linear combination experiment."
    echo "  --tohlinear      Run the linear toh combination experiment."
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
mkdir -p data/journal
mkdir -p data/journal/pancake
mkdir -p data/journal/road
mkdir -p data/journal/dao
mkdir -p data/journal/stp
mkdir -p data/journal/toh
mkdir -p data/journal/linear
mkdir -p data/journal/tohlinear

# Parse command-line arguments
stp_flag=false
toh_flag=false
pancake_flag=false
road_flag=false
dao_flag=false
linear_flag=false
toh_linear_flag=false

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -s|--stp) stp_flag=true ;;
        -t|--toh) toh_flag=true ;;
        -p|--pancake) pancake_flag=true ;;
        -r|--road) road_flag=true ;;
        -d|--dao) dao_flag=true ;;
        -l|--linear) linear_flag=true ;;
        --tohlinear) toh_linear_flag=true ;;
        -h|--help) usage ;;
        *) echo "Unknown option: $1"; usage ;;
    esac
    shift
done

algs=("bae" "tb-fd-fbi" "tb-df-fbi" "tb-g-fbi" "tb-b-fbi" "tb-gfgd-fbi" "tb-gdgf-fbi" "tb-gb-fbi")
nonroad_algs=("dbbs-max-a" "dbbs-max-p" "dbbs-max-fbi"
              "dbbsf-max-a" "dbbsf-max-p" "dbbsf-max-fbi"
              "dbbs-b-fbi" "dbbsf-b-fbi"
              "dbbs-ss-p" "dbbsf-ss-p" "gmxc")

all_algs=("${algs[@]}" "${nonroad_algs[@]}")

cd journal/bin/release || exit 1
for alg in "${all_algs[@]}"; do
  if $stp_flag; then
    ./aij -d stp -h md -i 0 -n 100 -a "$alg" | tee "../../../data/journal/stp/stp_${alg//\*/}.out"
  fi

  if $toh_flag; then
    for i in 2 4 6; do
      ./aij -d toh -h "$((12 - i))+$i" -i 0 -n 3 -a "$alg" | tee "../../../data/journal/toh/toh_${i}_${alg//\*/}.out"
    done
  fi

  if $pancake_flag; then
    for i in {0..5}; do
      ./aij -d pancake -h "gap-$i" -i 0 -n 50 -a "$alg" | tee "../../../data/journal/pancake/pancake_${i}_${alg//\*/}.out"
    done
  fi

  if $dao_flag; then
    ./aij -d dao -h all -i 0 -n 30 -a "$alg" | tee "../../../data/journal/dao/dao_${alg//\*/}.out"
  fi
done

if $road_flag; then
  for alg in "${algs[@]}"; do
    ./aij -d road -h eucspeed -i 0 -n 100 -a "$alg" | tee "../../../data/journal/road/road_${alg//\*/}.out"
  done
fi

if $linear_flag; then
  for i in {0..5}; do
      ./aij -d pancake -h "gap-$i" -i 50 -n 100 -a csbs -w 1 0 0 | tee "../../../data/journal/linear/pancake_${i}_fd.out"
      ./aij -d pancake -h "gap-$i" -i 50 -n 100 -a csbs -w 0 1 0 | tee "../../../data/journal/linear/pancake_${i}_df.out"
      ./aij -d pancake -h "gap-$i" -i 50 -n 100 -a csbs -w 0 0 1 | tee "../../../data/journal/linear/pancake_${i}_g.out"
  done
  # The weights are calculated as functions on the above results (in linear), but here are the calls with the weights of the paper
  ./aij -d pancake -h gap-0 -i 0 -n 50 -a csbs -w 0.49715347682084243 0.5028106011845583 0.000035921994599183276 | tee "../../../data/journal/linear/pancake_0_inv.out"
  ./aij -d pancake -h gap-0 -i 0 -n 50 -a csbs -w 0.4999638775196132 0.4999642839338045 0.00007183854658234308 | tee "../../../data/journal/linear/pancake_0_dif.out"
  ./aij -d pancake -h gap-1 -i 0 -n 50 -a csbs -w 0.4659580438072958 0.5335823399299067 0.00045961626279744877 | tee "../../../data/journal/linear/pancake_1_inv.out"
  ./aij -d pancake -h gap-1 -i 0 -n 50 -a csbs -w 0.4995077147277936 0.49957010518289907 0.0009221800893073559 | tee "../../../data/journal/linear/pancake_1_dif.out"
  ./aij -d pancake -h gap-2 -i 0 -n 50 -a csbs -w 0.5124034374928496 0.4603228317848368 0.027273730722313524 | tee "../../../data/journal/linear/pancake_2_inv.out"
  ./aij -d pancake -h gap-2 -i 0 -n 50 -a csbs -w 0.4760772121547877 0.47337060454123076 0.050552183303981535 | tee "../../../data/journal/linear/pancake_2_dif.out"
  ./aij -d pancake -h gap-3 -i 0 -n 50 -a csbs -w 0.31311818311617434 0.26469028091254043 0.4221915359712851 | tee "../../../data/journal/linear/pancake_3_inv.out"
  ./aij -d pancake -h gap-3 -i 0 -n 50 -a csbs -w 0.32903704771636516 0.29775755719222274 0.37320539509141215 | tee "../../../data/journal/linear/pancake_3_dif.out"
  ./aij -d pancake -h gap-4 -i 0 -n 50 -a csbs -w 0.17857576942915843 0.1835255362681431 0.6378986943026985 | tee "../../../data/journal/linear/pancake_4_inv.out"
  ./aij -d pancake -h gap-4 -i 0 -n 50 -a csbs -w 0.2780710517970431 0.2840565760503043 0.4378723721526526 | tee "../../../data/journal/linear/pancake_4_dif.out"
  ./aij -d pancake -h gap-5 -i 0 -n 50 -a csbs -w 0.13272495735150183 0.12430161967510664 0.7429734229733915 | tee "../../../data/journal/linear/pancake_5_inv.out"
  ./aij -d pancake -h gap-5 -i 0 -n 50 -a csbs -w 0.27742220313677785 0.2623391499380553 0.4602386469251668 | tee "../../../data/journal/linear/pancake_5_dif.out"
fi

if $toh_linear_flag; then
  for i in "10+2" "8+4" "6+6"; do
      ./aij -d toh -h "$i" -i 50 -n 50 -a csbs -w 1 0 0 | tee "../../../data/journal/tohlinear/toh_${i}_fd.out"
      ./aij -d toh -h "$i" -i 50 -n 50 -a csbs -w 0 1 0 | tee "../../../data/journal/tohlinear/toh_${i}_df.out"
      ./aij -d toh -h "$i" -i 50 -n 50 -a csbs -w 0 0 1 | tee "../../../data/journal/tohlinear/toh_${i}_g.out"
  done

./aij -d toh -h 10+2 -i 0 -n 50 -a csbs -w 0.5547498400249995 0.3994600037423087 0.045790156232691726 | tee "../../../data/journal/tohlinear/toh_10+2_inv.out"
./aij -d toh -h 10+2 -i 0 -n 50 -a csbs -w 0.4655262644645583 0.4521246205022131 0.08234911503322863 | tee "../../../data/journal/tohlinear/toh_10+2_dif.out"
./aij -d toh -h 8+4 -i 0 -n 50 -a csbs -w 0.463677529375743 0.39481111514016815 0.14151135548408875 | tee "../../../data/journal/tohlinear/toh_8+4_inv.out"
./aij -d toh -h 8+4 -i 0 -n 50 -a csbs -w 0.4082743643325132 0.39227477521340764 0.19945086045407914 | tee "../../../data/journal/tohlinear/toh_8+4_dif.out"
./aij -d toh -h 6+6 -i 0 -n 50 -a csbs -w 0.3800953821071525 0.37807119546363094 0.24183342242921646 | tee "../../../data/journal/tohlinear/toh_6+6_inv.out"
./aij -d toh -h 6+6 -i 0 -n 50 -a csbs -w 0.3602211052399233 0.3594727314542066 0.2803061633058701 | tee "../../../data/journal/tohlinear/toh_6+6_dif.out"
fi
