# Set up the Mask RCNN environment (Mask_RCNN folder)
1. Install dependencies
   ```bash
   pip install -r requirements.txt
   ```
2. Run setup from the repository root directory
    ```bash
    python setup.py install
    ``` 
3. Download pre-trained COCO weights (mask_rcnn_coco.h5) from the [releases page](https://github.com/matterport/Mask_RCNN/releases).
4. Install `pycocotools` from one of these repos. They are forks of the original pycocotools with fixes for Python3 and Windows (the official repo doesn't seem to be active anymore).

    * Linux: https://github.com/waleedka/coco
    * Windows: https://github.com/philferriere/cocoapi.
    You must have the Visual C++ 2015 build tools on your path (see the repo for additional details)

# Get bounding box with Mask RCNN (Mask_RCNN folder)
1. Run to get bounding box data. The output is in output_data.
   ```bash
   python mrcnn_data.py
   ```
2. (Optional) Run to get bounding box images. The output is in output_images.
   ```bash
   python mrcnn_images.py
   ```
3. (Optional) Run to Mask RCNN video. The output is in Mask_RCNN.
   ```bash
   python mrcnn_video.py
   ```

# Get landmark 3D location (under Landmark Detection folder)
Run matlab file “get_landmark_location.m”. The output is in output_landmarks.
