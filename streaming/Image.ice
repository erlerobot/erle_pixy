#ifndef IMAGE_ICE
#define IMAGE_ICE

module Image{  

  /**
   *  Static description of the image source.
   */
  sequence<byte> Data;
  class ImageDescription 
  {
    int width; /**< %Image width [pixels] */
    int height;/**< %Image height [pixels] */
	Data imageData;
  };
  
  interface ImageProvider
  {
    /**
     * Returns the latest data.
     */
	idempotent ImageDescription getImageData();
  };

}; //module

#endif //IMAGE_ICE
