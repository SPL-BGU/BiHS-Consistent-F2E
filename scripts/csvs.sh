cd "$(dirname "$0")"/.. || exit 1

mkdir -p results
mkdir -p results/csvs
mkdir -p results/csvs/conference
mkdir -p results/csvs/journal

echo "Converting conference STP logs to csv"
python3 analysis/agg_conference_logs.py -d stp -l data/conference/stp -o results/csvs/conference/stp.csv
echo "Converting conference Pancake logs to csv"
python3 analysis/agg_conference_logs.py -d pancake -l data/conference/pancake -o results/csvs/conference/pancake.csv
echo "Converting conference ToH logs to csv"
python3 analysis/agg_conference_logs.py -d toh -l data/conference/toh -o results/csvs/conference/toh.csv
echo "Converting conference DAO logs to csv"
python3 analysis/agg_conference_logs.py -d dao -l data/conference/dao -o results/csvs/conference/dao.csv
echo "Converting conference Road logs to csv"
python3 analysis/agg_conference_logs.py -d road -l data/conference/road -o results/csvs/conference/road.csv

echo "Converting journal STP logs to csv"
python3 analysis/agg_journal_logs.py -d stp -l data/journal/stp -o results/csvs/journal/stp.csv
echo "Converting journal Pancake logs to csv"
python3 analysis/agg_journal_logs.py -d pancake -l data/journal/pancake -o results/csvs/journal/pancake.csv
echo "Converting journal ToH logs to csv"
python3 analysis/agg_journal_logs.py -d toh -l data/journal/toh -o results/csvs/journal/toh.csv
echo "Converting journal DAO logs to csv"
python3 analysis/agg_journal_logs.py -d dao -l data/journal/dao -o results/csvs/journal/dao.csv
echo "Converting journal Road logs to csv"
python3 analysis/agg_journal_logs.py -d road -l data/journal/road -o results/csvs/journal/road.csv
echo "Converting journal Linear logs to csv"
python3 analysis/agg_journal_logs.py -d pancake -l data/journal/linear -o results/csvs/journal/linear.csv
