# Initial Setup

## Step1: **Install `pyenv`:**
   Use the `pyenv` yet, you can do so using a package manager or by cloning the GitHub repository. Here, I'll provide instructions for using the installation script:

   ```bash
   curl https://pyenv.run | bash
   ```

## Step2: **Add pyenv to shell configuration file(.bashrc):**

   ```bash
   echo 'export PATH="$HOME/.pyenv/bin:$PATH"' >> ~/.bashrc
   echo 'eval "$(pyenv init --path)"' >> ~/.bashrc
   echo 'eval "$(pyenv virtualenv-init -)"' >> ~/.bashrc
   exec "$SHELL"
   ```

## Step3: **Install dependencies:**

   ```bash
   sudo apt-get install --yes libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev llvm libncurses5-dev libncursesw5-dev xz-utils tk-dev libgdbm-dev lzma lzma-dev tcl-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev wget curl make build-essential openssl
   ```

   On other systems, the dependencies might be different. Refer to the [pyenv wiki](https://github.com/pyenv/pyenv/wiki/Common-build-problems) for more information.

## Step4: **Install a specific Python version:**
   Use the `pyenv install` command to install the Python version you want. For example, to install Python 3.9.12:

   ```bash
   pyenv install 3.9.12
   ```

   Replace `3.9.12` with the version you want to install.

## Step5: **Set the global or local Python version:**
   After the installation is complete, you can set the global Python version or set it locally in a specific directory. For example, to set the global version:

   ```bash
   pyenv global 3.9.12
   ```

   To set the version locally in a specific directory, navigate to the directory and run:

   ```bash
   pyenv local 3.9.12
   ```

   Adjust the version number accordingly.

## Step6: **Verify the installation:**
   You can verify that the correct Python version is being used by running:

   ```bash
   python --version
   ```

   It should display the version you installed.

## That's it! You have successfully installed a specific Python version using `pyenv`.

# Installation

Run the bash code below in your terminal to create and activate a new virtual environment named `.venv`. Ensure you are in the specific directory you want this environment to be installed.

curl https://pyenv.run | bash

```bash
python3 -m venv .venv
source .venv/bin/activate
```

## [Installation](#)

### Step 1: Install edge-tpu-silva

To install **edge-tpu-silva**, use the following pip command in a specified python environment:

```bash
pip install edge-tpu-silva
```

### Step 2: Run Setup Command

### [System Compatibility](#)

This table provides an overview of the compatibility of the system with different devices and operating systems.

|                | Compatibility | Setup Command        |
| -------------- | ------------- | -------------------- |
| Raspberry Pi 5 | ✔             | silvatpu-linux-setup |
| Raspberry Pi 4 | ✔             | silvatpu-linux-setup |
| Raspberry Pi 3 | ✔             | silvatpu-linux-setup |
| Jetson Nano    | ✔             | silvatpu-linux-setup |
| Windows        | ❌            |                      |
| macOS          | ❌            |                      |

In order to configure setup tools for your system, run the setup command in the terminal after step 1 is completed.

Example: If you are on a Raspberry Pi 5, run below command in the terminal following step 1.

```bash
silvatpu-linux-setup
```

The command installs the standard Edge TPU runtime for Linux, running the device at a reduced clock frequency. Alternatively, you can install a version for maximum speed, but be cautious of increased power consumption and device heat. If unsure, stick to the reduced frequency for safety. To install maximum frequency runtime, specify the speed of the setup command to `max`.

```bash
silvatpu-linux-setup --speed max
```

You cannot have both versions of the runtime installed at the same time, but you can switch by simply installing the alternate runtime as shown above

> **Caution:** Using the USB Accelerator at maximum clock frequency can make it dangerously hot. To prevent burn injuries, keep it out of reach or operate it at a reduced clock frequency.

> **Note:** Please ensure that you have the `Coral USB Accelerator` connected through `usb 3.0 port (for faster speed)`. If the Coral USB Accelerator was connected during the installation and setup, please disconnect and reconnect it to ensure `proper configuration.`

## [Models]()

To unleash the power of object `detection`, `segmentation`, and `classification` with this library, you'll need an Edge TPU-compatible .tflite model. These models should be exported using [`Ultralytics`](https://docs.ultralytics.com/modes/export/), ensuring a seamless integration with the edge-tpu-silva library.

> **NOTE:** Please be aware that the `imgsz` value specified during YOLO export should align with the same value used when defining `imgsz` for any of the processes. Consistency in these settings is crucial for optimal performance.

Smaller models will run faster but may have lower accuracy, while larger models will run slower but typically have higher accuracy. Explore the capabilities of edge computing with below models using edge-tpu-silva library.

| Download Link                                                                                                                                  | Process        | Base Model     | imgsz | Object Classes                                                                                    |
| ---------------------------------------------------------------------------------------------------------------------------------------------- | -------------- | -------------- | ----- | ------------------------------------------------------------------------------------------------- |
| [Download Model](https://github.com/DAVIDNYARKO123/edge-tpu-silva/blob/main/models/240_yolov8n_full_integer_quant_edgetpu.tflite?raw=true)     | Detection      | yolov8n.pt     | `240` | [COCO128](https://github.com/DAVIDNYARKO123/edge-tpu-silva/blob/main/asset/classes/coco128.txt)   |
| [Download Model](https://github.com/DAVIDNYARKO123/edge-tpu-silva/blob/main/models/240_yolov8n-seg_full_integer_quant_edgetpu.tflite?raw=true) | Segmentation   | yolov8n-seg.pt | `240` | [COCO128](https://github.com/DAVIDNYARKO123/edge-tpu-silva/blob/main/asset/classes/coco128.txt)   |
| [Download Model](https://github.com/DAVIDNYARKO123/edge-tpu-silva/blob/main/models/192_yolov8n_full_integer_quant_edgetpu.tflite?raw=true)     | Detection      | yolov8n.pt     | `192` | [COCO128](https://github.com/DAVIDNYARKO123/edge-tpu-silva/blob/main/asset/classes/coco128.txt)   |
| [Download Model](https://github.com/DAVIDNYARKO123/edge-tpu-silva/blob/main/models/192_yolov8n-seg_full_integer_quant_edgetpu.tflite?raw=true) | Segmentation   | yolov8n-seg.pt | `192` | [COCO128](https://github.com/DAVIDNYARKO123/edge-tpu-silva/blob/main/asset/classes/coco128.txt)   |
| [Download Model](https://github.com/DAVIDNYARKO123/edge-tpu-silva/blob/main/models/640_yolov8n-cls_full_integer_quant_edgetpu.tflite?raw=true) | Classification | yolov8n-cls.pt | `640` | [ImageNet](https://github.com/DAVIDNYARKO123/edge-tpu-silva/blob/main/asset/classes/imagenet.txt) |
| [Download Model](https://github.com/DAVIDNYARKO123/edge-tpu-silva/blob/main/models/240_yolov9c_full_integer_quant_edgetpu.tflite?raw=true)     | Detection      | yolov9c.pt     | `240` | [COCO128](https://github.com/DAVIDNYARKO123/edge-tpu-silva/blob/main/asset/classes/coco128.txt)   |

> **NOTE:** The YOLOv9 model, particularly the YOLOv9c.pt version, is substantial in size, which leads to its TensorFlow Lite version also being quite large. As a result, its processing speed on an Edge TPU is comparatively slower.

## Usage

### Object Detection Process

To perform object detection using the `process_detection` function, you can follow this example:

```python
from edge_tpu_silva import process_detection

# Run the object detection process
outs = process_detection(model_path='path/to/your/model.tflite', input_path='path/to/your/input/video.mp4', imgsz=192)

for _, _ in outs:
  pass
```

#### Running `process_detection` in the Terminal: Using the Entry Point "silvatpu"

To perform object detection with the `process_detection` function from the command line, you can use the user-friendly entry point `silvatpu`. Here's an example command:

```bash
silvatpu -p det -m path/to/model.tflite -i path/to/input/video.mp4 -z 192 -t 0.5 -v True
```

### Object Segmentation Process

To perform object segmentation using the `process_segmentation` function, you can follow this example:

```python
from edge_tpu_silva import process_segmentation

# Run the object segmentation process
outs = process_segmentation(model_path='path/to/your/model.tflite', input_path='path/to/your/input/video.mp4', imgsz=192)

for _, _ in outs:
  pass
```

#### Running `process_segmentation` in the Terminal: Using the Entry Point "silvatpu"

To perform object segmentation with the `process_segmentation` function from the command line, you can use the user-friendly entry point `silvatpu`. Here's an example command:

```bash
silvatpu -p seg -m path/to/model.tflite -i path/to/input/video.mp4 -z 192 -t 0.5 -v True
```

### Process `detection`, `segmentation` and `classification` Function Input Parameters

| Parameter    | Description                                            | Default Value |
| ------------ | ------------------------------------------------------ | ------------- |
| `model_path` | Path to the object segmentation model.                 | \-            |
| `input_path` | File path of image/video to process (Camera(0\|1\|2)). | \-            |
| `imgsz`      | Defines the image size for inference.                  | \-            |
| `threshold`  | Threshold for detected objects.                        | `0.4`         |
| `verbose`    | Display prints to the terminal.                        | `True`        |
| `show`       | Display frame with segmentation.                       | `False`       |
| `classes`    | Filters predictions to a set of class IDs.             | `None`        |

### Process `detection`, `segmentation` and `classification` Function Output

Each process function yields the following output:

| Output Parameter | Description                                     |
| ---------------- | ----------------------------------------------- |
| `objs_lst`       | List of objects detected in frame.              |
| `fps`            | Frames per second (fps) of the processed frame. |

Example usage:

```python
from edge_tpu_silva import process_detection

# Run the object detection process
outs = process_detection(model_path='path/to/your/model.tflite', input_path='path/to/your/input/video.mp4', imgsz=192)

for objs_lst, fps in outs:
    # Access the output parameters as needed
    print(f"Processed frame with {len(objs_lst)} objects. FPS: {fps}")
    print("List of object predictions in frame:")
    print(objs_lst)
```
