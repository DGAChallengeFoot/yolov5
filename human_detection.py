import time
import threading
from pathlib import Path

import torch
import torch.backends.cudnn as cudnn

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, non_max_suppression, scale_coords, \
    xyxy2xywh, set_logging, increment_path
from utils.torch_utils import select_device, time_synchronized


class DetectionArgs:
    def __init__(self):
        # Args defined by user
        self.weights = 'yolov5s.pt'
        self.source = '0'
        self.imgsz = 640
        self.conf_thres = 0.25
        self.iou_thres = 0.45
        self.device = ''
        self.view_img = False
        self.save_txt = True
        self.save_conf = True
        self.classes = [0]
        self.agnostic_nms = False
        self.augment = False
        self.update = False
        self.project = 'runs/detect'
        self.name = 'exp'
        self.exist_ok = True

        # Args self-defined, do not set manually
        self.model = None
        self.webcam = None
        self.save_dir = None
        self.half = None
        self.dataset = None
        self.stopFlag = False


defaultArgs: DetectionArgs = DetectionArgs()
finalArgs: DetectionArgs = DetectionArgs()


def setup(args: DetectionArgs = defaultArgs):
    args.webcam = args.source.isnumeric() or args.source.endswith('.txt') or args.source.lower().startswith(
        ('rtsp://', 'rtmp://', 'http://'))

    # Directories
    args.save_dir = Path(increment_path(Path(args.project) / args.name, exist_ok=args.exist_ok))  # increment run
    (args.save_dir / 'labels' if args.save_txt else args.save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Initialize
    set_logging()
    args.device = select_device(args.device)
    args.half = args.device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    args.model = attempt_load(args.weights, map_location=args.device)  # load FP32 model
    args.imgsz = check_img_size(args.imgsz, s=args.model.stride.max())  # check img_size
    if args.half:
        args.model.half()  # to FP16

    # Set Dataloader
    if args.webcam:
        cudnn.benchmark = True  # set True to speed up constant image size inference
        args.dataset = LoadStreams(args.source, img_size=args.imgsz)
    else:
        args.dataset = LoadImages(args.source, img_size=args.imgsz)

    global finalArgs
    finalArgs = args


def runDetection():
    global finalArgs
    lock = threading.Lock()
    thread = threading.Thread(target=detect, args=(finalArgs, lock,))
    thread.start()
    return thread, lock


def stopDetection(lock: threading.Lock):
    lock.acquire()
    finalArgs.stopFlag = True
    lock.release()


def detect(args: DetectionArgs = defaultArgs, lock: threading.Lock = None):
    # Run inference
    t0 = time.time()
    img = torch.zeros((1, 3, args.imgsz, args.imgsz), device=args.device)  # init img
    _ = args.model(img.half() if args.half else img) if args.device.type != 'cpu' else None  # run once
    try:
        for path, img, im0s, _ in args.dataset:
            img = torch.from_numpy(img).to(args.device)
            img = img.half() if args.half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # Inference
            t1 = time_synchronized()
            pred = args.model(img, augment=args.augment)[0]

            # Apply NMS
            pred = non_max_suppression(pred, args.conf_thres, args.iou_thres, classes=args.classes,
                                       agnostic=args.agnostic_nms)
            t2 = time_synchronized()
            # Process detections
            for i, det in enumerate(pred):  # detections per image
                if args.webcam:  # batch_size >= 1
                    p, s, im0, frame = path[i], '%g: ' % i, im0s[i].copy(), args.dataset.count
                else:
                    p, s, im0, frame = path, '', im0s, getattr(args.dataset, 'frame', 0)

                p = Path(p)  # to Path
                txt_path = str(args.save_dir / 'labels' / p.stem) + (
                    '' if args.dataset.mode == 'image' else f'_{frame}')  # img.txt
                s += '%gx%g ' % img.shape[2:]  # print string
                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                    # Print results
                    for c in det[:, -1].unique():
                        n = (det[:, -1] == c).sum()  # detections per class
                        s += f'{n} Humans, '  # add to string

                    # Write results
                    for *xyxy, conf, cls in reversed(det):
                        if args.save_txt:  # Write to file
                            # TODO send data to main thread
                            xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                            line = (cls, *xywh, conf) if args.save_conf else (cls, *xywh)  # label format
                            with open(txt_path + '.txt', 'a') as f:
                                f.write(('%g ' * len(line)).rstrip() % line + '\n')

                # Print time (inference + NMS)
                print(f'{s}Done. ({t2 - t1:.3f}s)')

                if lock:
                    lock.acquire()
                    if finalArgs.stopFlag:
                        lock.release()
                        args.dataset.stop()
                    lock.release()

    except StopIteration:
        print("Iteration interrupted, detection done.")


if __name__ == '__main__':
    with torch.no_grad():
        setup()
        t, l = runDetection()
        time.sleep(2)
        stopDetection(l)
        t.join()
