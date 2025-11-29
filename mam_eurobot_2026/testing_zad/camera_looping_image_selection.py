import cv2
import time
import os

def run_timelapse(interval=5, total_duration=100, output_folder='dataset'):
    # 1. Create the output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        print(f"Created folder: {output_folder}")

    cap = cv2.VideoCapture(0) # Change 0 to appropriate camera index if needed
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # 2. Initialize Timers
    start_time = time.time()
    last_capture_time = start_time
    img_counter = 0
    
    print(f"Starting time-lapse. Duration: {total_duration}s. Interval: {interval}s.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        current_time = time.time()
        elapsed_total = current_time - start_time
        time_since_last_capture = current_time - last_capture_time

        # 3. Check if it is time to stop
        if elapsed_total > total_duration:
            print("Time limit reached. Finishing up.")
            break

        # 4. Check if it is time to capture (Non-blocking timer)
        if time_since_last_capture >= interval:
            img_name = f"{output_folder}/img_{img_counter}.jpg"
            cv2.imwrite(img_name, frame)
            print(f"Captured {img_name} at {int(elapsed_total)}s")
            
            # Reset the interval timer and increment counter
            last_capture_time = current_time
            img_counter += 1

        # --- VISUAL FEEDBACK ON SCREEN ---
        # Calculate countdown for the display
        next_shot_in = interval - time_since_last_capture
        display_frame = frame.copy()
        
        # Draw: "Next photo in: 3.2s"
        cv2.putText(display_frame, f"Next photo: {next_shot_in:.1f}s", (10, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        # Draw: "Total Progress: 15/100s"
        cv2.putText(display_frame, f"Total Time: {int(elapsed_total)}/{total_duration}s", (10, 90), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

        cv2.imshow('Time Lapse Monitor', display_frame)

        # Allow quitting early with 'q'
        if cv2.waitKey(1) == ord('q'):
            print("Time-lapse cancelled by user.")
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"Done! Saved {img_counter} images to '{output_folder}'.")

if __name__ == "__main__":
    # Run: Every 5 seconds, for 100 seconds total
    run_timelapse(interval=5, total_duration=100)