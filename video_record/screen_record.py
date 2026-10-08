import cv2
import numpy as np
import mss
import time
import os
from datetime import datetime
folder = './screen_record_v1/'
os.makedirs(folder,exist_ok=True)
def screen_record(show_fps=True, target_fps=30):
    # Initialize screen capture
    sct = mss.mss()
    monitor = sct.monitors[1]  # primary screen
    width, height = monitor["width"], monitor["height"]

    # Temporary filename (will rename later)
    date_str = datetime.now().strftime("%m-%d-%H-%M")
    temp_filename = f"{folder}/{date_str}-{target_fps}.avi"

    # Video writer
    fourcc = cv2.VideoWriter_fourcc(*"XVID")
    out = cv2.VideoWriter(temp_filename, fourcc, 30, (width, height))
    # Tracking variables
    prev_time = time.time()
    fps_display = 0
    frame_count_sec = 0
    total_frames = 0
    start_time = time.time()

    print(f"Recording started ({temp_filename}) — Press Ctrl+C to stop.\n")

    try:
        while True:
            current_time = time.time()
            # Capture screen
            img = np.array(sct.grab(monitor))
            frame = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            #frame = np.array(sct.grab(monitor),dtype=np.uint8)[:, :, :3]
            # Frame counters
            total_frames += 1
            frame_count_sec += 1

            # Update FPS display every second
            if current_time - prev_time >= 1:
                fps_display = frame_count_sec
                frame_count_sec = 0
                prev_time = current_time

            # Overlay FPS text
            if show_fps:
                cv2.putText(frame, f"FPS: {fps_display}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)


            # Write and display
            out.write(frame)
            #cv2.imshow("Screen Recorder", frame)

            # Allow window to refresh
            # if cv2.waitKey(1) & 0xFF == 27:  # ESC backup
            #     break

            # Control capture rate
            elapsed = time.time() - current_time
            delay = max(1.0 / target_fps - elapsed, 0)
            time.sleep(delay)

    except KeyboardInterrupt:
        print("\nCtrl+C detected — stopping recording...")

    finally:
        # Calculate real FPS
        end_time = time.time()
        duration = end_time - start_time
        true_fps = total_frames / duration if duration > 0 else 0

        # Cleanup
        out.release()
        cv2.destroyAllWindows()
        sct.close()

        # Rename file with true FPS
        new_filename = f"{folder}/{date_str}-{true_fps:.2f}.avi"
        os.rename(temp_filename, new_filename)

        print("\n✅ Recording finished.")
        print(f"📁 Saved file: {new_filename}")
        print(f"🕒 Duration: {duration:.2f} seconds")
        print(f"🎞️ Total frames: {total_frames}")
        print(f"⚙️ True average FPS: {true_fps:.2f}")

if __name__ == "__main__":
    screen_record(show_fps=True, target_fps=1000)
