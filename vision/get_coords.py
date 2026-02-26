import cv2
import numpy as np
import sympy as sp


def detect_aruco_markers():
    # Load a predefined ArUco dictionary (6x6 grid with 250 possible IDs)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    # Create default parameters for ArUco marker detection
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict)

    # Open the camera (camera index 0)
    cap = cv2.VideoCapture(0)

    detected_markers = []  # Store details of detected markers (id, x_center, y_center)

    # Main loop to capture and process frames from the camera
    while True:
        # Capture a single frame from the camera
        ret, frame = cap.read()

        # Check if the frame was successfully captured
        if not ret:
            print("Failed to grab frame.")
            break

        # Convert the captured frame to grayscale for ArUco detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers in the frame
        corners, ids, _ = detector.detectMarkers(gray)

        # If markers are detected, process them
        if ids is not None:
            detected_markers = []  # Reset the list for the current frame
            for i, corner in enumerate(corners):
                # Draw a polygon around the detected marker
                cv2.polylines(frame, [np.int32(corner)], True, (0, 255, 0), 2)

                # Calculate the center of the marker
                x_center = int((corner[0][0][0] + corner[0][2][0]) / 2)  # X-coordinate of the center
                y_center = int((corner[0][0][1] + corner[0][2][1]) / 2)  # Y-coordinate of the center

                # Retrieve the ID of the marker
                marker_id = ids[i][0]

                # Append the detected marker details to the list
                detected_markers.append((marker_id, x_center, y_center))

                # Display the marker ID and its center coordinates on the frame
                cv2.putText(frame, f"ID: {marker_id} X: {x_center} Y: {y_center}", (x_center, y_center),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Print the detected markers
            for marker in detected_markers:
                print(f"ID: {marker[0]} X: {marker[1]} Y: {marker[2]}")

        # Display the frame with the detected markers
        cv2.imshow('ArUco Marker Detection', frame)

        # Break the loop if two markers are detected
        # if len(detected_markers) >= 2:
        #     print("Detected two markers. Exiting...")

        # Exit the loop when the user presses the 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera resource
    cap.release()

    # Close all OpenCV windows
    cv2.destroyAllWindows()

    # Return the detected markers (two or fewer)
    return detected_markers[:2]


# Call the function to start detection
markers = detect_aruco_markers()
# mx = 0.00018603454307636466
# cx = -0.4731453235859212
# my = -0.00017721101959803707
# cy = -0.15661624804175914
# # mx=0.00047067890835579524
# # cx= -0.45739058288409706
# # my= -0.0004580623306233063
# # cy= -0.15854525745257453
z_fixed = 0.157

mx=0.00019559492862156975
cx= -0.5029680194855188
my= -0.0001950830851201598
cy= -0.16842472094980854

# mx=-2.4961972979083174e-05
# cx= -0.26697716836967006
# my= -7.175417637631128e-06
# cy= -0.26585909682605235
# #
if markers:
    with open('marker_coords.txt', 'w') as file:
        for i, marker in enumerate(markers):
            print(f"Marker {i + 1}: ID: {marker[0]}, X: {marker[1]}, Y: {marker[2]}")
            # Calculate the real-world x and y coordinates of the marker
            x_m = sp.N(mx * marker[1] + cx)
            y_m = sp.N(my * marker[2] + cy)

            # Write the real-world coordinates with the fixed z-value to the file
            file.write(f"{x_m},{y_m},{z_fixed}\n")
            print(f"Real-world coordinates: X: {x_m}, Y: {y_m}, Z: {z_fixed}")
else:
    print("No markers detected.")
