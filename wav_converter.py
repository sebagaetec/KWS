import os
import wave
import struct

def convert_wav_to_c_header(wav_path, header_path, variable_name="g_test_audio_sample"):
    """
    Converts a .wav file into a C header file containing the raw audio data
    as an int16_t array. The audio is expected to be 16kHz, 16-bit, mono.
    """
    try:
        with wave.open(wav_path, 'rb') as wav_file:
            # --- Verify WAV file properties ---
            num_channels = wav_file.getnchannels()
            sample_width = wav_file.getsampwidth()
            frame_rate = wav_file.getframerate()
            num_frames = wav_file.getnframes()

            print(f"Reading '{wav_path}':")
            print(f"  - Channels: {num_channels}")
            print(f"  - Sample Width: {sample_width} bytes ({sample_width*8}-bit)")
            print(f"  - Frame Rate: {frame_rate} Hz")
            print(f"  - Number of Frames: {num_frames}")

            if frame_rate != 16000:
                print(f"Warning: Frame rate is {frame_rate}Hz, but 16000Hz is expected.")
            if num_channels != 1:
                print("Warning: Audio is not mono. Only the first channel will be considered if stereo.")
            if sample_width != 2: # 2 bytes = 16 bits
                print(f"Error: Sample width is not 2 bytes (16-bit). Got {sample_width} bytes.")
                return

            # --- Read audio data ---
            frames = wav_file.readframes(num_frames)
            # Unpack the binary data into a tuple of short integers (h)
            samples = struct.unpack(f'<{num_frames}h', frames)

            # --- Write to C header file ---
            with open(header_path, 'w') as header_file:
                header_file.write(f"/*\n * Audio sample data from: {os.path.basename(wav_path)}\n */\n\n")
                header_file.write("#ifndef AUDIO_SAMPLE_H\n")
                header_file.write("#define AUDIO_SAMPLE_H\n\n")
                header_file.write("#include <stdint.h>\n\n")
                header_file.write(f"#define {variable_name.upper()}_LEN {len(samples)}\n\n")
                header_file.write(f"const int16_t {variable_name}[{len(samples)}] = {{\n    ")

                for i, sample in enumerate(samples):
                    header_file.write(f"{sample}, ")
                    if (i + 1) % 16 == 0:
                        header_file.write("\n    ")

                header_file.write("\n};\n\n")
                header_file.write("#endif /* AUDIO_SAMPLE_H */\n")

            print(f"\nSuccessfully converted to '{header_path}'")
            print(f"Variable name: {variable_name}")
            print(f"Length: {len(samples)} samples")

    except Exception as e:
        print(f"An error occurred: {e}")


input_wav_file = 'right.wav'  # Change to your WAV file name
output_header_file = 'audio_sample.h'
c_variable_name = 'g_right_audio_sample'

# Check if the input file exists
if not os.path.exists(input_wav_file):
    print(f"Error: Input file '{input_wav_file}' not found.")
    print("Please make sure the WAV file is in the same directory as this script,")
    print("or provide the full path.")
else:
    convert_wav_to_c_header(input_wav_file, output_header_file, c_variable_name)