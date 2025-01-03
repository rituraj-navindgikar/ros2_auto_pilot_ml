import os
import pandas as pd

def merge_csv_files_in_cwd(output_file="merged_output.csv"):
    cwd = os.getcwd()
    csv_files = [file for file in os.listdir(cwd) if file.endswith(".csv")]

    if not csv_files:
        print("No CSV files found in the current working directory.")
        return

    print(f"Found {len(csv_files)} CSV files: {csv_files}")

    dataframes = []

    for csv_file in csv_files:
        print(f"Merging: {csv_file}")
        df = pd.read_csv(csv_file)
        dataframes.append(df)

    merged_data = pd.concat(dataframes, ignore_index=True)

    merged_data.to_csv(output_file, index=False)
    print(f"Merged file saved as: {output_file}")


if __name__ == "__main__":
    merge_csv_files_in_cwd()
