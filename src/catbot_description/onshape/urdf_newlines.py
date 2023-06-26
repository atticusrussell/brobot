import argparse
import re

def format_urdf(input_file_name, output_file_name=None):
    with open(input_file_name, 'r') as file:
        content = file.read()

    formatted_content = re.sub(r'(\s*<link name)', r'\n\1', content)
    formatted_content = re.sub(r'(\s*<joint name)', r'\n\1', formatted_content)

    if output_file_name is None:
        output_file_name = input_file_name

    with open(output_file_name, 'w') as file:
        file.write(formatted_content)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process URDF files.')
    parser.add_argument('input_file', type=str, help='Input URDF file name')
    parser.add_argument('output_file', type=str, nargs='?', default=None, help='Output URDF file name (optional)')

    args = parser.parse_args()

    format_urdf(args.input_file, args.output_file)
