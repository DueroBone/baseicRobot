import base64
import zlib


def zip_and_base64_encode_file(file_path):
    try:
        # Read the content of the file
        with open(file_path, "rb") as file:
            file_content = file.read()

        # Compress the file content using zlib
        # compressed_data = zlib.compress(file_content)

        # Encode the compressed data in base64
        # encoded_data = base64.b64encode(compressed_data).decode("utf-8")
        encoded_data = base64.b64encode(file_content).decode("utf-8")

        return encoded_data

    except Exception as e:
        print(f"Error: {e}")
        return None


# Example usage:
file_path = "rawModel"  # Replace with the path to your file
encoded_data = zip_and_base64_encode_file(file_path)

if encoded_data is not None:
    # print("Base64 encoded and compressed data:")
    print(encoded_data)
else:
    print("Failed to generate encoded data.")
