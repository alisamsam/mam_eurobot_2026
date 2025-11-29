import cv2
import cv2.aruco as aruco
import numpy as np
from tkinter import Tk
from tkinter.filedialog import askopenfilename

# THIS CODE SECTION IS USED TO MANUALLY TEST ON PRESELECTED IMAGES
# REMOVE GET_IMAGE_PATH AND CHANGE MAIN TO BETTER FIT OUR PURPOSE IF INCORPORATING INTO ROBOT

def get_image_path():
    root = Tk()
    root.withdraw() 
    file_path = askopenfilename(title="Select Image", filetypes=[("Images", "*.jpg *.png *.bmp")])
    root.destroy()
    return file_path


def detect_markers_in_tile(img_tile, offset_x, offset_y):
    """
    Detects markers in a specific crop (tile) of the image.
    Returns the corners adjusted to global coordinates and the IDs.
    """
    gray = cv2.cvtColor(img_tile, cv2.COLOR_BGR2GRAY)
    
    # Use the 4x4 dictionary (standard for these crates)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    # --- AGGRESSIVE TUNING ---
    parameters.minMarkerPerimeterRate = 0.01  # Look for small markers
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 30
    parameters.adaptiveThreshWinSizeStep = 2
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    # -------------------------

    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)

    global_corners = []
    found_ids = []

    if ids is not None:
        for i in range(len(ids)):
            # Offset the corners by the tile's position
            # corners[i] is shape (1, 4, 2)
            c = corners[i][0]
            c[:, 0] += offset_x
            c[:, 1] += offset_y
            
            global_corners.append(np.array([c]))
            found_ids.append(ids[i][0])
            
    return global_corners, found_ids

def main():
    image_path = get_image_path()
    if not image_path: 
        return

    full_img = cv2.imread(image_path)
    if full_img is None: 
        return
    
    h, w = full_img.shape[:2]
    
    # Dictionary to store unique markers (ID -> Corners) to avoid duplicates from overlaps
    unique_markers = {}

    # --- SLIDING WINDOW CONFIGURATION ---
    # We will split the image into a grid (e.g., 2 rows, 3 columns)
    rows = 2
    cols = 3
    
    # Calculate tile size with some overlap
    tile_h = int(h / rows)
    tile_w = int(w / cols)
    overlap = 50 # pixels overlap to ensure we don't cut a marker in half

    print("Scanning image tiles...")
    
    for r in range(rows):
        for c in range(cols):
            # Define the crop coordinates
            y1 = max(0, r * tile_h - overlap)
            y2 = min(h, (r + 1) * tile_h + overlap)
            x1 = max(0, c * tile_w - overlap)
            x2 = min(w, (c + 1) * tile_w + overlap)

            # Crop the tile
            tile = full_img[y1:y2, x1:x2]
            
            # Detect in this specific tile
            corners, ids = detect_markers_in_tile(tile, x1, y1)
            
            # Store results (deduplicate by ID)
            for i, marker_id in enumerate(ids):
                if marker_id not in unique_markers:
                    unique_markers[marker_id] = corners[i]

    # --- VISUALIZATION ---
    # Reconstruct lists for drawing
    if unique_markers:
        final_ids = np.array([[k] for k in unique_markers.keys()], dtype=np.int32)
        final_corners = list(unique_markers.values())
        
        print(f"\nSuccess! Found {len(final_ids)} unique markers after tiling.")

        # Draw everything on the original image
        aruco.drawDetectedMarkers(full_img, final_corners, final_ids)
        
        label_map = { 36: "Blue crate", 47: "Yellow crate", 20: "Yellow nest quadrant", 21: "Blue nest quadrant", 22: "Yellow cursor quadrant", 23: "Blue cursor quadrant" }

        for i in range(len(final_ids)):
            m_id = final_ids[i][0]
            c = final_corners[i][0]
            cx, cy = int(np.mean(c[:, 0])), int(np.mean(c[:, 1]))

            label = label_map.get(m_id, "Other")
            text = f"{label} ({m_id})"
            
            # Draw text
            cv2.putText(full_img, text, (cx, cy - 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("Tiled Detection Result", full_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("Still no markers found. The image resolution might be too low for these markers.")

if __name__ == "__main__":
    main()