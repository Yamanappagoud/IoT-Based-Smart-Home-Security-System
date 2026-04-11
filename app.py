from flask import Flask, request, jsonify
import cv2
import numpy as np
import os
import requests
import threading
import io
import time
import logging
import sys
import traceback

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    stream=sys.stdout
)
logger = logging.getLogger(__name__)

app = Flask(__name__)

# Telegram Bot Configuration
TELEGRAM_BOT_TOKEN = "7555720722:AAHnCQW2M70jIFH1Ol08lO9UDqp2RBHqmSc"
TELEGRAM_CHAT_ID = "7081127777"  # The chat ID where notifications will be sent

# Synchronous function to send message and image to Telegram
def send_telegram_notification(image, detections):
    try:
        # Create message with detections
        if detections:
            message = "🔍 Object Detection Results:\n"
            for i, detection in enumerate(detections, 1):
                message += f"{i}. {detection['class']}\n"
        else:
            message = "No objects detected in the image."
        
        # Draw bounding boxes on the image
        img_with_boxes = draw_boxes(image.copy(), detections)
        
        # Convert numpy array to bytes
        is_success, buffer = cv2.imencode(".jpg", img_with_boxes)
        if not is_success:
            raise Exception("Failed to encode image")
        
        # API endpoint for sending photo
        url = f"https://api.telegram.org/bot{TELEGRAM_BOT_TOKEN}/sendPhoto"
        
        # First send the image
        files = {'photo': ('image.jpg', buffer.tobytes(), 'image/jpeg')}
        data = {'chat_id': TELEGRAM_CHAT_ID, 'caption': "Processed Image"}
        
        response = requests.post(url, files=files, data=data)
        response.raise_for_status()
        logger.info("Image sent to Telegram successfully")
        
        # Send the detection results text
        url = f"https://api.telegram.org/bot{TELEGRAM_BOT_TOKEN}/sendMessage"
        data = {'chat_id': TELEGRAM_CHAT_ID, 'text': message}
        
        response = requests.post(url, data=data)
        response.raise_for_status()
        logger.info("Message sent to Telegram successfully")
        
        return True
    except Exception as e:
        logger.error(f"Error sending to Telegram: {str(e)}")
        logger.error(traceback.format_exc())
        return False

def notify_telegram_thread(image, detections):
    """Send Telegram notification in a separate thread"""
    def thread_function():
        try:
            send_telegram_notification(image, detections)
        except Exception as e:
            logger.error(f"Error in telegram thread: {str(e)}")
            logger.error(traceback.format_exc())
    
    thread = threading.Thread(target=thread_function)
    thread.daemon = True
    thread.start()

def draw_boxes(img, detections):
    try:
        height, width, _ = img.shape
        
        # Load class information
        classes = []
        with open("coco.names", "r") as f:
            classes = [line.strip() for line in f.readlines()]
        
        colors = np.random.uniform(0, 255, size=(len(classes), 3))
        
        # Draw bounding boxes based on detection results
        for detection in detections:
            class_name = detection["class"]
            
            # If we have box coordinates, use them
            if "box" in detection:
                x, y, w, h = detection["box"]
                try:
                    class_idx = classes.index(class_name)
                    color = colors[class_idx].tolist()
                    
                    cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                    cv2.putText(img, class_name, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                except ValueError:
                    # If class name is not found in classes list
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(img, class_name, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return img
    except Exception as e:
        logger.error(f"Error drawing boxes: {str(e)}")
        return img  # Return original image if drawing fails

# Load YOLO model
def load_yolo():
    try:
        # Check if all required files exist
        required_files = ["yolov3.weights", "yolov3.cfg", "coco.names"]
        for file in required_files:
            if not os.path.exists(file):
                download_yolo_files()
                break
                
        net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
        classes = []
        with open("coco.names", "r") as f:
            classes = [line.strip() for line in f.readlines()]
        layers_names = net.getLayerNames()
        
        # Handle different versions of OpenCV
        try:
            output_layers = [layers_names[i - 1] for i in net.getUnconnectedOutLayers()]
        except:
            output_layers = [layers_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
            
        logger.info("YOLO model loaded successfully")
        return net, classes, output_layers
    except Exception as e:
        logger.error(f"Error loading YOLO model: {str(e)}")
        logger.error(traceback.format_exc())
        raise

# Download required files if they don't exist
def download_yolo_files():
    urls = {
        "yolov3.weights": "https://pjreddie.com/media/files/yolov3.weights",
        "yolov3.cfg": "https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov3.cfg",
        "coco.names": "https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names"
    }
    
    current_dir = os.getcwd()
    logger.info(f"Current working directory: {current_dir}")
    logger.info(f"Directory contents: {os.listdir(current_dir)}")
    
    for filename, url in urls.items():
        filepath = os.path.join(current_dir, filename)
        if not os.path.exists(filepath):
            logger.info(f"Downloading {filename} from {url}")
            try:
                response = requests.get(url, stream=True)
                response.raise_for_status()
                
                with open(filepath, 'wb') as f:
                    if filename == "yolov3.weights":
                        # For weights file, download in chunks due to large size
                        for chunk in response.iter_content(chunk_size=8192):
                            if chunk:
                                f.write(chunk)
                    else:
                        f.write(response.content)
                logger.info(f"Successfully downloaded {filename}")
            except Exception as e:
                logger.error(f"Error downloading {filename}: {str(e)}")
                logger.error(traceback.format_exc())
                raise

# Health check endpoint
@app.route('/health', methods=['GET'])
def health_check():
    try:
        # Check if YOLO files exist
        files_status = {}
        for file in ["yolov3.weights", "yolov3.cfg", "coco.names"]:
            files_status[file] = os.path.exists(file)
        
        return jsonify({
            "status": "healthy",
            "files": files_status,
            "working_directory": os.getcwd()
        })
    except Exception as e:
        return jsonify({
            "status": "unhealthy",
            "error": str(e)
        }), 500

@app.route('/detect', methods=['POST'])
def detect_objects():
    try:
        logger.info("Received detection request")
        
        # Detailed request debugging
        logger.info(f"Request content type: {request.content_type}")
        logger.info(f"Request headers: {request.headers}")
        logger.info(f"Request files: {request.files.keys()}")
        logger.info(f"Request form data: {request.form}")
        
        # Check if image is in request
        if 'image' not in request.files:
            logger.warning("No image provided in request")
            return jsonify({"error": "No image provided"}), 400
        
        file = request.files['image']
        logger.info(f"Received file: {file.filename}, Content-Type: {file.content_type}")
        
        if file.filename == '':
            logger.warning("Empty filename in request")
            return jsonify({"error": "No selected file"}), 400

        # Read image
        logger.info("Reading image data")
        file_bytes = file.read()
        logger.info(f"Received image data size: {len(file_bytes)} bytes")
        
        # Save a copy of the raw image for debugging
        try:
            debug_image_path = os.path.join(os.getcwd(), 'last_received_image.jpg')
            with open(debug_image_path, 'wb') as f:
                f.write(file_bytes)
            logger.info(f"Saved debug copy of received image to {debug_image_path}")
        except Exception as e:
            logger.error(f"Failed to save debug image: {str(e)}")
        
        # Try to decode the image
        img_array = np.frombuffer(file_bytes, np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        
        if img is None:
            logger.error("Failed to decode image, it might be corrupted")
            return jsonify({"error": "Failed to decode image"}), 400
            
        height, width, _ = img.shape
        logger.info(f"Successfully decoded image. Size: {width}x{height}")
        
        # Try to enhance the image for better detection
        try:
            # Apply some basic image enhancements
            img_enhanced = img.copy()
            
            # Apply clahe for better contrast
            lab = cv2.cvtColor(img_enhanced, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
            cl = clahe.apply(l)
            limg = cv2.merge((cl,a,b))
            img_enhanced = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
            
            # Save enhanced image for debugging
            cv2.imwrite('enhanced_image.jpg', img_enhanced)
            logger.info("Saved enhanced image for debugging")
            
            # Use enhanced image for detection
            img = img_enhanced
        except Exception as e:
            logger.warning(f"Image enhancement failed: {str(e)}")
            # Continue with original image

        # Load YOLO model
        logger.info("Loading YOLO model")
        net, classes, output_layers = load_yolo()

        # Detecting objects - try with lower confidence threshold
        logger.info("Processing image with YOLO")
        blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        # Information to display on screen
        class_ids = []
        confidences = []
        boxes = []

        # Lower the confidence threshold for detection
        confidence_threshold = 0.3  # Was 0.5, now more sensitive

        # Showing information on the screen
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > confidence_threshold:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
                    
                    logger.debug(f"Found object: {classes[class_id]} with confidence {confidence:.2f}")

        # Apply non-max suppression
        logger.info("Applying non-max suppression")
        if len(boxes) > 0:
            indexes = cv2.dnn.NMSBoxes(boxes, confidences, confidence_threshold, 0.4)
        else:
            indexes = []

        # Prepare results
        results = []
        logger.info(f"Found {len(indexes)} objects after non-max suppression")
        
        # Check if indexes is a numpy array or list
        if isinstance(indexes, np.ndarray):
            indexes_list = indexes.flatten()
        else:
            # For older OpenCV versions
            indexes_list = [i[0] for i in indexes] if len(indexes) > 0 else []
            
        for i in range(len(boxes)):
            if i in indexes_list:
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                confidence_val = confidences[i]
                results.append({
                    "class": label,
                    "confidence": float(confidence_val),
                    "box": [x, y, w, h]  # Include box coordinates for drawing
                })
                logger.info(f"Detected {label} with confidence {confidence_val:.2f}")
        
        # Send notification to Telegram bot (non-blocking)
        if results:
            logger.info(f"Sending notification to Telegram with {len(results)} detections")
            try:
                notify_telegram_thread(img, results)
            except Exception as e:
                logger.error(f"Failed to send Telegram notification: {str(e)}")
        else:
            logger.warning("No objects detected to send to Telegram")
            try:
                # Send the raw image to telegram anyway
                message = "No objects were detected in this image."
                thread = threading.Thread(target=lambda: send_telegram_notification(img, []))
                thread.daemon = True
                thread.start()
            except Exception as e:
                logger.error(f"Failed to send empty notification: {str(e)}")

        # Simplify response to just include class name and confidence
        api_results = []
        for result in results:
            api_results.append({
                "class": result["class"],
                "confidence": result["confidence"]
            })
            
        logger.info(f"Returning detection results: {api_results}")
        return jsonify({"detections": api_results})

    except Exception as e:
        logger.error(f"Error in detection endpoint: {str(e)}")
        logger.error(traceback.format_exc())
        return jsonify({
            "error": str(e), 
            "traceback": traceback.format_exc()
        }), 500

# Add a new health check endpoint for the ESP32-CAM to test connectivity
@app.route('/esp32test', methods=['GET', 'POST'])
def esp32_test():
    """Simple endpoint for ESP32-CAM to test connectivity"""
    if request.method == 'POST':
        return jsonify({"status": "success", "message": "POST request received successfully"})
    else:
        return jsonify({"status": "success", "message": "GET request received successfully"})

if __name__ == '__main__':
    try:
        # Download required files
        download_yolo_files()
        
        # Run the app
        port = int(os.environ.get("PORT", 5000))
        logger.info(f"Starting Flask app on port {port}")
        app.run(debug=False, host='0.0.0.0', port=port)
    except Exception as e:
        logger.error(f"Error starting app: {str(e)}")
        logger.error(traceback.format_exc())