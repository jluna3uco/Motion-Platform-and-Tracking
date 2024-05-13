import cv2
import matplotlib.pyplot as plt
import numpy as np

def main():
    # Load the video file
    video_path = 'Motion_Capture_Video2.mp4'
    cap = cv2.VideoCapture(video_path)

    # Select the tracker type
    tracker_type = "CSRT"

    # Initialize the tracker
    if tracker_type == 'CSRT':
        tracker = cv2.TrackerCSRT_create()
    else:
        raise ValueError('Invalid tracker type specified')

    # Read the first frame
    ret, frame = cap.read()

    # Select the region of interest (ROI) to track
    bbox = cv2.selectROI("Select Object to Track", frame, fromCenter=False, showCrosshair=True)

    # Initialize the tracker with the first frame and the bounding box
    tracker.init(frame, bbox)

    # Lists to store x-axis positions and timestamps
    x_positions = []
    timestamps = []

    # Loop through the video frames
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Update the tracker and get the new bounding box
        success, bbox = tracker.update(frame)
        
        # Record x-axis position and timestamp
        if success:
            x_positions.append(bbox[0] + bbox[2] / 2)  # Center of the bounding box
            timestamps.append(cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0)  # Convert milliseconds to seconds
        else:
            x_positions.append(None)  # Tracking failure
            timestamps.append(cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0)  # Convert milliseconds to seconds
        
        # Display the resulting frame
        cv2.imshow('Object Tracking', frame)
        
        # Exit if ESC pressed
        if cv2.waitKey(1) & 0xFF == 27:
            break

    # Release the video capture object and close windows
    cap.release()
    cv2.destroyAllWindows()

    # Plotting the positions versus time
    plt.plot(timestamps, x_positions, marker='o', linestyle='-')
    plt.xlabel('Time (s)')
    plt.ylabel('X Position (pixels)')
    plt.title('X Position vs Time')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()