#!/bin/zsh

# alias bt='./scripts/preproccess_bt.zsh src/igvc25/config/bt_raw.xml src/igvc25/config/bt_clean.xml'

# Check for required argument
if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <input_file> [output_file]"
  exit 1
fi

input_file="$1"
output_file="${2:-cleaned_${input_file}}"

# Use sed to remove key="" or key="" followed by space, while preserving line structure
sed -E 's/ [a-zA-Z0-9_:-]+=""//g' "$input_file" > "$output_file"

echo "Cleaned file written to: $output_file"
