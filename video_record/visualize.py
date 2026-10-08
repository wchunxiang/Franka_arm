import imageio
import cv2
import time
from datetime import datetime
import numpy as np
import time

camera_source = '<video4>'
fps = 1000

# Generate video filename with the current timestamp
timestamp = datetime.now().strftime("%Y%m%d_%H%M_%S")

# Define parameters



if __name__ == '__main__':
    # Open the camera using imageio
    camera = imageio.get_reader(camera_source)  # Use "<video0>" to capture from the first camera device

    # Initialize variables
    frame_size = None
    fourcc = cv2.VideoWriter_fourcc(*"XVID")  # Codec for AVI format
    video_writer = None

    print("Recording started... Press 'q' to stop.")
    cv2.namedWindow("Visulaize", cv2.WINDOW_NORMAL) 
    start_time = time.time()
    #frame_timestamps = []
    try:
        current_time = time.time()
        for frame in camera:
            # Dynamically set frame size and initialize video writer
            #frame_timestamps.append(frame_time)

            # Convert frame to BGR format for OpenCV
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Display the frame (optional)
            cv2.imshow("Visulaize", frame_bgr)

            # Check for key press to stop recording
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Recording stopped by user.")
                break

            # Wait to maintain frame rate
            elapsed = time.time() - current_time
            delay = max(1.0 / fps - elapsed, 0)
            time.sleep(delay)
            current_time = time.time()
            #time.sleep(1 / fps)

    finally:
        # Release resources
        camera.close()
        cv2.destroyAllWindows()
        print(f'FPS: {1/elapsed}')
    # frame_intervals = [frame_timestamps[i] - frame_timestamps[i-1] for i in range(1, len(frame_timestamps))]
    # print(frame_intervals)
    # print(np.mean(frame_intervals))
