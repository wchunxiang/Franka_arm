import imageio
import cv2
import time
from datetime import datetime
import os
import time

camera_source = '<video4>'
fps = 20

folder = '../../data_fluoro/'
os.makedirs(folder, exist_ok=True)

# Generate video filename with the current timestamp
timestamp = datetime.now().strftime("%Y%m%d_%H%M_%S")
video_filename = folder + f"{timestamp}.avi"

# Define parameters


if __name__ == '__main__':
    # Open the camera using imageio
    camera = imageio.get_reader(camera_source)  # Use "<video0>" to capture from the first camera device

    # Initialize variables
    frame_size = None
    fourcc = cv2.VideoWriter_fourcc(*"XVID")  # Codec for AVI format
    video_writer = None

    print("Recording started... Press 'q' to stop.")

    start_time = time.time()
    frame_count = 0
    current_time = time.time()
    try:
        for frame in camera:
            # Dynamically set frame size and initialize video writer
            if frame_size is None:
                frame_size = (frame.shape[1], frame.shape[0])  # Width, Height
                video_writer = cv2.VideoWriter(video_filename, fourcc, fps, frame_size)
                print(f"Initialized video writer with resolution {frame_size[0]}x{frame_size[1]}")

            # Convert frame to BGR format for OpenCV
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Display the frame (optional)
            cv2.imshow("Recording", frame_bgr)

            # Write the frame to the video file
            video_writer.write(frame_bgr)
            frame_count += 1

            # Check for key press to stop recording
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Recording stopped by user.")
                break

            # Wait to maintain frame rate
            # Wait to maintain frame rate
            elapsed = time.time() - current_time
            delay = max(1.0 / fps - elapsed, 0)
            time.sleep(delay)
            current_time = time.time()

    finally:
        # Release resources
        camera.close()
        if video_writer:
            video_writer.release()
        cv2.destroyAllWindows()

    print(f"Recording completed. Video saved as '{video_filename}'.")
    print(f"Total frames captured: {frame_count}. Time: {time.time()-start_time} s")
