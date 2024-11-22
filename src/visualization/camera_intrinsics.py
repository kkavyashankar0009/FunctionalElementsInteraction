import zipfile
import os

# Path to the zip file
zip_path = r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\42445198\hires_wide_intrinsics.zip"
extract_dir = r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\42445198"  # Directory where files will be extracted

# Extract the zip file
with zipfile.ZipFile(zip_path, 'r') as zip_ref:
    zip_ref.extractall(extract_dir)

print(f"Extracted files to {extract_dir}")

# Locate the .pincam file
hires_intrinsics_dir = os.path.join(extract_dir, "hires_wide_intrinsics")
pincam_file_name = "42445198_10879.256.pincam"  # Replace with the correct file name
pincam_file_path = os.path.join(hires_intrinsics_dir, pincam_file_name)

# Check if the file exists
if not os.path.isfile(pincam_file_path):
    raise FileNotFoundError(f"The file {pincam_file_path} does not exist.")

# Parse the file content
with open(pincam_file_path, 'r') as file:
    content = file.read().strip()  # Remove extra whitespace or newline characters

# Split the content into components
width, height, fx, fy, cx, cy = map(float, content.split())

# Construct the intrinsic matrix
intrinsic_matrix = [
    [fx,  0, cx],
    [ 0, fy, cy],
    [ 0,  0,  1]
]

# Print the intrinsic matrix
print("Intrinsic Matrix:")
for row in intrinsic_matrix:
    print(row)

# Optionally, print the image dimensions
print(f"Image Width: {width}")
print(f"Image Height: {height}")
