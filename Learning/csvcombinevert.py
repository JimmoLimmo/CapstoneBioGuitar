"""
stack_csvs.py
-------------
Combines two or more CSV files vertically (row-wise) so that:
  - Rows from file1 come first, then file2, then file3, etc.
  - Headers are handled automatically (only one header row in the output)

Usage:
    python stack_csvs.py file1.csv file2.csv [file3.csv ...] output.csv

Optional flags:
    --no-header         Treat all files as having no header row
    --allow-mismatch    Allow files with different columns (missing cells filled
                        with the --fill value). Default: error on mismatch.
    --fill FILL         Fill value for missing cells when columns differ
                        (default: empty string). Only used with --allow-mismatch.
    --add-source        Add a '__source' column with the originating filename.

Examples:
    python stack_csvs.py jan.csv feb.csv mar.csv q1.csv
    python stack_csvs.py a.csv b.csv out.csv --allow-mismatch --fill "N/A"
    python stack_csvs.py a.csv b.csv out.csv --add-source
"""

import argparse
import csv
import sys


def read_csv(path: str):
    with open(path, newline="", encoding="utf-8-sig") as f:
        return list(csv.reader(f))


def stack_csvs(files: list, output: str, has_header: bool, allow_mismatch: bool,
               fill: str, add_source: bool):

    all_data = []
    header = None

    for path in files:
        rows = read_csv(path)
        if not rows:
            print(f"Warning: {path} is empty, skipping.")
            continue

        if has_header:
            file_header = rows[0]
            data_rows   = rows[1:]

            if header is None:
                header = file_header
            elif file_header != header:
                if not allow_mismatch:
                    print(
                        f"Error: header in '{path}' does not match the first file.\n"
                        f"  Expected : {header}\n"
                        f"  Got      : {file_header}\n"
                        f"Use --allow-mismatch to combine anyway."
                    )
                    sys.exit(1)
                else:
                    print(f"Warning: header mismatch in '{path}', columns will be aligned by name.")
        else:
            data_rows = rows

        all_data.append((path, data_rows))

    if not all_data:
        print("No data to combine.")
        sys.exit(1)

    if has_header and allow_mismatch:
        seen = []
        headers_per_file = {}
        # Re-read headers
        for path in files:
            rows = read_csv(path)
            if rows:
                h = rows[0]
                headers_per_file[path] = h
                for col in h:
                    if col not in seen:
                        seen.append(col)
        unified_header = seen
    else:
        unified_header = header 

    with open(output, "w", newline="", encoding="utf-8") as fout:
        writer = csv.writer(fout)

        if has_header and unified_header is not None:
            out_header = unified_header + (["__source"] if add_source else [])
            writer.writerow(out_header)

        total_rows = 0
        for path, data_rows in all_data:
            for row in data_rows:
                if has_header and allow_mismatch and unified_header:
                    file_cols = headers_per_file.get(path, unified_header)
                    row_dict  = dict(zip(file_cols, row))
                    aligned   = [row_dict.get(col, fill) for col in unified_header]
                else:
                    aligned = row

                if add_source:
                    aligned = list(aligned) + [path]

                writer.writerow(aligned)
                total_rows += 1

    print(f"Done! Stacked file written to: {output}")
    print(f"  Files stacked : {len(all_data)}")
    print(f"  Data rows     : {total_rows}")
    if has_header and unified_header:
        print(f"  Columns       : {len(unified_header)}{' + __source' if add_source else ''}")


def main():
    parser = argparse.ArgumentParser(
        description="Stack two or more CSV files vertically (row-wise)."
    )
    parser.add_argument("files", nargs="+", help="Input CSV files followed by the output path")
    parser.add_argument(
        "--no-header",
        action="store_true",
        help="Treat all files as having no header row",
    )
    parser.add_argument(
        "--allow-mismatch",
        action="store_true",
        help="Allow files with different columns (aligned by header name)",
    )
    parser.add_argument(
        "--fill",
        default="",
        help="Fill value for missing cells when columns differ (default: '')",
    )
    parser.add_argument(
        "--add-source",
        action="store_true",
        help="Add a '__source' column with the originating filename",
    )

    args = parser.parse_args()

    if len(args.files) < 2:
        print("Error: provide at least one input file and one output file.")
        sys.exit(1)

    *input_files, output = args.files

    if len(input_files) < 1:
        print("Error: provide at least one input CSV file.")
        sys.exit(1)

    stack_csvs(
        files=input_files,
        output=output,
        has_header=not args.no_header,
        allow_mismatch=args.allow_mismatch,
        fill=args.fill,
        add_source=args.add_source,
    )


if __name__ == "__main__":
    main()