import cv2
from ultralytics import YOLO

# Load trained model
model = YOLO("best 65.pt")

# Read an image (replace with your image path), "C:\Users\Admin\Downloads\detection_11_1767420864.jpg","C:\Users\Admin\Downloads\detection_4_1767420854.jpg","C:\Users\Admin\Downloads\detection_1_1767421115 (1).jpg"
image_path = r"C:\Users\Admin\Downloads\Actor047_a10_f0072.jpg"
image = cv2.imread(image_path)

# Check if the image is read properly
if image is None:
    print("Failed to read image. Check the path.")
else:
    # Run YOLO prediction
    results = model.predict(source=image, show=False, save=False, verbose=False, conf=0.5)

    # Draw results on the image
    annotated_image = results[0].plot()

    # Create a resizable window
    cv2.namedWindow("YOLOv8 Inference - Image", cv2.WINDOW_NORMAL)

    # Get screen size
    screen_res = 1280, 720  # You can adjust this based on your monitor (width, height)
    scale_width = screen_res[0] / annotated_image.shape[1]
    scale_height = screen_res[1] / annotated_image.shape[0]
    scale = min(scale_width, scale_height)

    # Resize image to fit screen
    window_width = int(annotated_image.shape[1] * scale)
    window_height = int(annotated_image.shape[0] * scale)
    resized_image = cv2.resize(annotated_image, (window_width, window_height))

    # Show the image
    cv2.imshow("YOLOv8 Inference - Image", resized_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
