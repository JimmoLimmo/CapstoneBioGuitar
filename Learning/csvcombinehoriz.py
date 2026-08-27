"""
combine_csvs.py
---------------
Combines two CSV files side-by-side (horizontally) so that:
  - The first N columns come from file1.csv
  - The remaining columns come from file2.csv
  - Rows are matched by position (row 1 with row 1, etc.)

Usage:
    python csvcombinehoriz.py file1.csv file2.csv output.csv

Optional flags:
    --no-header     Treat both files as having no header row
    --fill FILL     Value to use when one file has more rows than the other
                    (default: empty string)

Examples:
    python csvcombinehoriz.py employees.csv salaries.csv combined.csv
    python csvcombinehoriz.py data1.csv data2.csv out.csv --fill "N/A"
"""

import argparse
import csv
import sys
from itertools import zip_longest


def combine_csvs(file1: str, file2: str, output: str, has_header: bool, fill: str):
    with open(file1, newline="", encoding="utf-8-sig") as f1, \
         open(file2, newline="", encoding="utf-8-sig") as f2:

        reader1 = csv.reader(f1)
        reader2 = csv.reader(f2)

        rows1 = list(reader1)
        rows2 = list(reader2)

    if not rows1 and not rows2:
        print("Both files are empty. Nothing to combine.")
        sys.exit(1)

    max_cols1 = max((len(r) for r in rows1), default=0)
    max_cols2 = max((len(r) for r in rows2), default=0)

    combined_rows = []

    for row1, row2 in zip_longest(rows1, rows2, fillvalue=None):

        left  = row1 if row1 is not None else [fill] * max_cols1
        right = row2 if row2 is not None else [fill] * max_cols2

        left  = left  + [fill] * (max_cols1 - len(left))
        right = right + [fill] * (max_cols2 - len(right))

        combined_rows.append(left + right)

    with open(output, "w", newline="", encoding="utf-8") as fout:
        writer = csv.writer(fout)
        writer.writerows(combined_rows)

    total_rows = len(combined_rows)
    data_rows  = total_rows - (1 if has_header else 0)
    total_cols = max_cols1 + max_cols2

    print(f"Done! Combined file written to: {output}")
    print(f"  Rows : {total_rows} ({data_rows} data row(s){', plus 1 header' if has_header else ''})")
    print(f"  Cols : {total_cols} ({max_cols1} from file1 + {max_cols2} from file2)")


def main():
    parser = argparse.ArgumentParser(
        description="Combine two CSV files side-by-side (horizontally)."
    )
    parser.add_argument("file1",  help="Path to the first CSV file (left columns)")
    parser.add_argument("file2",  help="Path to the second CSV file (right columns)")
    parser.add_argument("output", help="Path for the combined output CSV file")
    parser.add_argument(
        "--no-header",
        action="store_true",
        help="Treat files as having no header row (default: assumes header present)",
    )
    parser.add_argument(
        "--fill",
        default="",
        help="Fill value for missing rows/cells when file lengths differ (default: '')",
    )

    args = parser.parse_args()
    has_header = not args.no_header

    combine_csvs(args.file1, args.file2, args.output, has_header, args.fill)


if __name__ == "__main__":
    main()