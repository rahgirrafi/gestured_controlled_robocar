import numpy as np
import cv2
from imgTransferServer import VideoStreamReceiver
import time

def save_image(image, filename):
    """
    Save the image to the specified filename.
    """
    cv2.imwrite(filename, image)
    print(f"Image saved as {filename}")


if __name__ == "__main__":

    receiver = VideoStreamReceiver()
    receiver.start()

    run_version = '1'
   
    saved_image_count = 0
    current_time = time.time() 
    while receiver.running:
        if receiver.rgb_frame is not None:
            img_rgb = receiver.rgb_frame
            # save image once after every 1 second 
            print(time.time() - current_time )
            if time.time() - current_time > 0.5:
                img_name = f'{run_version}_{saved_image_count}.jpg'
                cv2.imwrite(img_name, receiver.current_frame)
                saved_image_count += 1
                print(f"Image saved at {run_version}_{saved_image_count}.jpg")
                current_time = time.time()

            cv2.imshow('Hand Landmarks', receiver.current_frame)
            

        if cv2.waitKey(1) & 0xFF == ord('q'):
            receiver.stop()
            break