cd "$(dirname "$0")"/.. || exit 1

echo "Generating bound progression figure"
python3 analysis/bounds_progress_figure.py

mkdir -p "results/tables"
python3 analysis/latex_tables.py

