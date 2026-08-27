import subprocess
import sys
import numpy as np
import cv2
import pytesseract
from music21 import note
from midiutil import MIDIFile
import matplotlib.pyplot as plt
import os

# Ensure that an argument is provided when running this script
if len(sys.argv) != 2:
    print("Usage: python execute_main.py <image_file>")
    sys.exit(1)

# Get the image file argument from the command line
image_file = sys.argv[1]

# Load and preprocess the image
image = cv2.imread(image_file)

if image is None:
    print('Could not open or find the image')
    sys.exit(1)

# Adjust contrast
alpha = 1.4  # Contrast control
beta = 0     # Brightness control
new_image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

# Convert to grayscale
gray_image = cv2.cvtColor(new_image, cv2.COLOR_BGR2GRAY)

# Thresholding the image
_, thresh_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

# Display thresholded image for debugging
plt.imshow(thresh_image, cmap='gray')
plt.title('Thresholded Image')
plt.show()

# Save the thresholded image to a temporary file
temp_file = "temp_thresholded_image.png"
cv2.imwrite(temp_file, thresh_image)

# Command to execute the main.py script with the thresholded image file as an argument
command = ["python", "main.py", temp_file]

try:
    # Run the command and capture the output
    result = subprocess.run(command, capture_output=True, text=True, check=True)
    print("Output from main.py:")
    print(result.stdout)

    # Use the output directly as `detected_notes_with_positions` without parsing
    detected_notes_with_positions = eval(result.stdout)  # Assuming `main.py` outputs a valid Python list
except subprocess.CalledProcessError as e:
    print("Error occurred while executing main.py:")
    print(e.stderr)
    sys.exit(1)
finally:
    # Clean up the temporary file
    if os.path.exists(temp_file):
        os.remove(temp_file)

# Overlay detected notes onto the original image
#if detected_notes_with_positions:
    #annotated_image = image.copy()
    #font = cv2.FONT_HERSHEY_SIMPLEX
    #font_scale = 0.6
    #font_color = (0, 255, 255)  
    #font_thickness = 1

    #for note_name, x, y in detected_notes_with_positions:
        #cv2.putText(annotated_image, note_name, (x, y), font, font_scale, font_color, font_thickness)

    # Save the annotated image
    #annotated_image_path = "annotated_image.png"
    #cv2.imwrite(annotated_image_path, annotated_image)
    #print(f"Annotated image saved to {annotated_image_path}")
#else:
    #print("No notes detected. No annotated image will be created.")
    #sys.exit(1)

# Convert detected notes to a MIDI file
midi_file = MIDIFile(1)
midi_file.addTempo(0, 0, 120)

time = 0  # Start time
for n, _, _ in detected_notes_with_positions:
    try:
        # Convert symbolic note name to MIDI pitch
        midi_note = note.Note(n)
        midi_file.addNote(0, 0, midi_note.pitch.midi, time, 1, 100)
        time += 1  # Move to the next note
    except Exception as e:
        print(f"Error processing note {n}: {e}")

# Save MIDI file
output_midi_path = "output_music.mid"
with open(output_midi_path, "wb") as output_file:
    midi_file.writeFile(output_file)

print("MIDI file created:", output_midi_path)
