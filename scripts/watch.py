import cv2, sys

# Open the video
video_path = sys.argv[1]
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# Get total frame count
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
print("Total frames:", total_frames)

# Function to jump to a specific frame
def goto_frame(frame_num):
    if 0 <= frame_num < total_frames:
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_num)
    else:
        print("Frame number out of range")

# Start at frame 0
current_frame = 0
goto_frame(current_frame)

while True:
    ret, frame = cap.read()
    if not ret:
        print("End of video or failed to grab frame.")
        break

    cv2.imshow("Video", frame)
    key = cv2.waitKey(0) & 0xFF  # wait for a key press

    if key == ord('q'):  # quit
        break
    elif key == ord('n') or key == 83:  # next frame (right arrow or n)
        current_frame += 1
        goto_frame(current_frame)
    elif key == ord('b') or key == 81:  # back one frame (left arrow or b)
        current_frame = max(0, current_frame - 1)
        goto_frame(current_frame)
    elif key == ord('s'):  # skip forward 10 frames
        current_frame = min(total_frames - 1, current_frame + 10)
        goto_frame(current_frame)
    elif key == ord('r'):  # rewind 10 frames
        current_frame = max(0, current_frame - 10)
        goto_frame(current_frame)

cap.release()
cv2.destroyAllWindows()
