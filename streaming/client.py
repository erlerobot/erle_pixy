import sys, traceback, Ice
import Image
import numpy as np
import cv2
status = 0
ic = None
try:
    ic = Ice.initialize(sys.argv)
    base = ic.stringToProxy("ImageServer:default -h localhost -p 10000")
    im = Image.ImageProviderPrx.checkedCast(base)
    if not im:
        raise RuntimeError("Invalid proxy")

    while 1:
        imagen = im.getImageData()
        print imagen.width, imagen.height
        #print imagen.imageData
        img = np.frombuffer(imagen.imageData, dtype=np.uint8)
        img.shape = imagen.width, imagen.height, 3
        cv2.imshow('recibido',img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
except:
    traceback.print_exc()
    status = 1

if ic:
    # Clean up
    try:
        ic.destroy()
    except:
        traceback.print_exc()
        status = 1

sys.exit(status)
