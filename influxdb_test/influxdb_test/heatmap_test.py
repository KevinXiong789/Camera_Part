
import cv2
import numpy as np

# Create a VideoCapture object for the D435 camera
#the first usb port on the front of the chassis
cap = cv2.VideoCapture(4)

# Define the size of the motion heatmap
heatmap_size = (640, 480)

# Create a motion heatmap array
heatmap = np.zeros(heatmap_size, dtype=np.float32)

# Define decay factor and threshold value
decay_factor = 0.05
threshold_value = 10

# Initialize previous frame
prev_gray = None

# Process live video frames
while True:
    # Get a frame from the camera
    ret, frame = cap.read()
    
    # Resize the frame to the heatmap size
    frame = cv2.resize(frame, heatmap_size)
    
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # If this is the first frame, set the previous frame to the current frame
    if prev_gray is None:
        prev_gray = gray
        
        continue
    

    # Calculate the absolute difference between the current frame and the previous frame
    diff = cv2.absdiff(gray, prev_gray)
    
    # Apply a threshold to the difference image
    threshold = 25
    _, thresh = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)
    
    # Blur the thresholded image
    kernel_size = (5, 5)
    blur = cv2.GaussianBlur(thresh, kernel_size, 0)
    
    # Create a binary image of the motion areas
    _, binary = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY)
    
    # Transpose the binary array to match the shape of the heatmap array
    binary = np.transpose(binary)
    
    # Update the motion heatmap
    heatmap += binary
    heatmap *= (1 - decay_factor) # apply decay factor to reduce heatmap intensity over time
    
    # Threshold the heatmap to remove noise and only show active regions
    _, heatmap_thresh = cv2.threshold(heatmap, threshold_value, 255, cv2.THRESH_BINARY)

    # Apply a color map to the heatmap
    heatmap_color = cv2.applyColorMap(np.uint8(heatmap * 10), cv2.COLORMAP_HOT)
    
    # Rotate the heatmap 90 degrees clockwise
    heatmap_color = cv2.rotate(heatmap_color, cv2.ROTATE_90_CLOCKWISE)
    
    # Mirror the heatmap horizontally
    heatmap_color = cv2.flip(heatmap_color, 1)

    # Display the video and the motion heatmap
    cv2.imshow('Video', frame)
    cv2.imshow('Heatmap', heatmap_color)
    
    # Set the previous frame to the current frame
    prev_gray = gray
    
    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and destroy all windows
cap.release()
cv2.destroyAllWindows()




