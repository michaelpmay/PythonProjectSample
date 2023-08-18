import numpy as np
from cellpose import models
import scipy.signal as signal
class iMicroscopyImage:
    def shape(self):
        '''return the size of the image'''
        pass

    def getChannel(self,index):
        '''return the numerical values of pixels in a channel'''
        pass

    def setChannel(self, index, pixels):
        '''sets the value of pixels'''
        pass

    def addMeta(self, key, value):
        '''adds a key-value relationship to the metadata of the image'''
        pass

    def acceptProcess(self, process):
        '''acceps a process object and implements processing on itself'''
        pass

    def copy(self):
        '''returns a copy of itself which does not have strange array property'''
        pass

class iMicroscopyImageStack:
    def append(self,image):
        '''appends an image to the end of the stack using a copy of the image instead of the origional'''
        pass
    def remove(self,index):
        '''removes an image at the indexed value'''
        pass
    def acceptProcess(self, process):
        '''accepts a process object and implements the processing on each elemet in the stack and chanel'''
        pass
    def getImageFromIndex(self,index):
        '''gets image from the stack associated with the index. returns an image'''
        pass


class iMicroscopyImageProcess:
    def append(self):
        '''appends a processor to the end of the stack of processes'''
        pass
    def process(self,image):
        '''Processes and image and returns the updated image. Must always return an image object.'''
        return image


class iMicroscopyImageProcessLibrary:
    'A factory design pattern for differnt image process nodes'
    pass


class MicroscopyImage(iMicroscopyImage):
    channel=None
    meta = None
    shape=None

    def __init__(self):
        self.channel = []


    @property
    def shape(self):
        if len(self.channel)==0:
            return ()
        else:
            shape=()
            for i in range(len(self.channel)):
                if len(self.channel[i])!=0:
                    shape=self.channel[i].shape
                    break
        return shape+(len(self.channel),)

    def getChannel(self,index):
        return self.channel[index]

    def setChannel(self, index, pixels):
        if (type(pixels) is np.ndarray):
            pass
        elif (type(pixels) is list):
            pixels=np.array(pixels)
        else:
            raise TypeError
        if len(self.channel)>0:
            if (pixels.shape[0]!=self.shape[0])|(pixels.shape[1]!=self.shape[1]):
                raise ValueError
        numChannels=len(self.channel)
        if index>=numChannels:
            numExtensions=index-numChannels+1
            for i in range(numExtensions):
                self.channel.append([])
        pixelCopy=np.copy(pixels)
        self.channel[index]=pixelCopy

    def addMeta(self,key,value):
        self.meta[key]=value

    def acceptProcess(self,process):
        self=process(self)
        return self

    def copy(self):
        clone=MicroscopyImage()
        for i in range(len(self.channel)):
            clone.setChannel(i,np.copy(self.channel[i]))
        clone.meta=self.meta
        return clone

class MicroscopyImageStack(iMicroscopyImageStack):
    stack=None
    def __init__(self):
        self.stack=[]
    def __len__(self):
        return len(self.stack)
    def append(self,image):
        if not isinstance(image,MicroscopyImage):
            raise TypeError
        self.stack.append(image.copy())
    def acceptProcess(self,process):
        for i in range(len(self)):
            image=self.stack[i]
            self.stack[i]=image.acceptProcess(process)
        return self
    def getImageFromIndex(self,index):
        return self.stack[index]

    def remove(self,index):
        # todo
        pass


class MicroscopyImageProcess(iMicroscopyImageProcess):
    processor=None
    property=None
    def __init__(self,processor=None):
        if processor is not None:
            if not callable(processor):
                raise TypeError
        self.processor=processor
        self.property={'meta':None,'bridge':None,'stack':None}
    def process(self,image):
        image=self.processor(image,self.property)
        return image
    def __call__(self, image):
        return self.processor(image,self.property)

class MicroscopyImageProcessLibrary:
    '''A collection of useful image processes.'''
    @staticmethod
    def null():
        '''process that does nothing'''
        def processor(image,property):
            return image

        node=MicroscopyImageProcess(processor=processor)
        return node

    @staticmethod
    def printString(string):
        '''process that does nothing'''
        if not isinstance(string,str):
            raise TypeError

        def processor(image, property):
            print(string)
            return image

        node = MicroscopyImageProcess(processor=processor)
        return node

    @staticmethod
    def doEvery(step):
        if not isinstance(step, int):
            raise TypeError
        def processor(image, property):
            return image

        node = MicroscopyImageProcess(processor=processor)
        return node

    @staticmethod
    def channelMap(self):
        # todo
        def processor(image, property):
            return image

        node = MicroscopyImageProcess(processor=processor)
        return node

    @staticmethod
    def detectCentroids(self):
        # todo
        def process(image, property):
            return image

        return process

    @staticmethod
    def add(value):
        '''add a constant to all channels'''
        if not (isinstance(value,int)|isinstance(value,float)):
            raise TypeError
        def processor(image,property):
            for i in range(len(image.channel)):
                image.channel[i]=image.channel[i]+value
            return image

        node = MicroscopyImageProcess(processor=processor)
        return node

    @staticmethod
    def addChannel(value,index):
        '''add a constant to a channel'''
        def processor(image,property):
            image.channel[index] = image.channel[index] + value
            return image

        node = MicroscopyImageProcess(processor=processor)
        return node

    @staticmethod
    def subtract(value):
        '''subtract a constant from the pixels in all channels'''
        if not (isinstance(value,int)|isinstance(value,float)):
            raise TypeError
        def processor(image,property):
            for i in range(len(image.channel)):
                image.channel[i] = image.channel[i] - value
            return image

        node = MicroscopyImageProcess(processor=processor)
        return node

    @staticmethod
    def subtractChannel(value,index):
        '''subtract a constant from all channels'''
        if not (isinstance(value,int)|isinstance(value,float)):
            raise TypeError
        def processor(image,property):
            image.channel[index] = image.channel[index] - value
            return image

        node = MicroscopyImageProcess(processor=processor)
        return node

    @staticmethod
    def multiply(value):
        '''multiplies each pixel in all channels against a scalar'''
        if not (isinstance(value,int)|isinstance(value,float)):
            raise TypeError
        def processor(image,property):
            for i in range(len(image.channel)):
                image.channel[i]=image.channel[i]*value
            return image

        node = MicroscopyImageProcess(processor=processor)
        return node

    @staticmethod
    def multiplyChannel(value,index):
        '''ultiplies each pichel in a channel against a scalar'''
        if not (isinstance(value,int)|isinstance(value,float)):
            raise TypeError
        def processor(image,property):
            image.channel[index] = image.channel[index] * value
            return image

        node = MicroscopyImageProcess(processor=processor)
        return node

    @staticmethod
    def divide(value):
        '''multiplies each pixel in all channels against a scalar'''
        if not (isinstance(value,int)|isinstance(value,float)):
            raise TypeError
        def processor(image, property):
            for i in range(len(image.channel)):
                image.channel[i] = image.channel[i] * value
            return image

        node = MicroscopyImageProcess(processor=processor)
        return node

    @staticmethod
    def crop(xyrange):
        '''crops all all channels to a particular size'''
        for i in range(4):
            if not isinstance(xyrange[i],int):
                raise TypeError
        def processor(image,property):
            for i in range(len(image.channel)):
                image.channel[i]=image.channel[i][xyrange[0]:(xyrange[1]+1),xyrange[2]:(xyrange[3]+1)]
            return image

        node = MicroscopyImageProcess(processor=processor)
        return node


    @staticmethod
    def astype(type):
        '''changes the format of pixels to a type (uint8, uint16, single, double, ect)'''
        def processor(image,property):
            for i in range(len(image.channel)):
                image.channel[i] = image.channel[i].astype(type)
            return image

        node = MicroscopyImageProcess(processor=processor)
        return node

    @staticmethod
    def pseudocolor(colormaps):
        '''changes the channel visualization to another color using colormaps'''
        def process(image,property):
            # todo
            return image
        return process

class MicroscopyImageProcessPipeline:
    stack=None
    def __init__(self):
        self.stack=[]

    def append(self,node):
        self.stack.append(node)

    def process(self,image):
        for i in range(len(self.stack)):
            image=self.stack[i](image)
        return image

    def __call__(self,image):
        return self.process(image)

class MicrosopyImageProcessBuilder:
    pipeline=None
    def __init__(self):
        self.pipeline=MicroscopyImageProcessPipeline()

    def add(self,node):
        self.pipeline.append(node)

    def getPipeline(self):
        return self.pipeline

class SpotCountLocations(iMicroscopyImageProcess):
    def process(self, image, sig=1, threshhold=50,maxNum=1000):
        #use convolutions to highlight spots with certain size and count them
        print(image)
        print(sig)
        image = np.array(image)
        kernelSize=round(sig*4)
        ax = np.linspace(-(kernelSize - 1) / 2., (kernelSize - 1) / 2., kernelSize)
        kernel = np.exp(-0.5 * np.square(ax) / np.square(sig))
        kernel = np.outer(kernel, kernel)
        kernel = kernel / np.sum(kernel)
        kernel = kernel - np.mean(kernel)
        if len(image.shape)==2:
            image=np.expand_dims(image,2)
        spotLocations=[]
        for i in range(image.shape[2]):
            spot_image = signal.convolve2d(image[:, :, i], kernel, boundary='symm', mode='same')
            spot_image=spot_image/(image.shape[2]+1)
            output=self.spotCountFromSpotImage(spot_image, threshhold,maxNum,sig)
            print('Spot Locations{0}'.format(output))
            spotLocations.append(output)
        return spotLocations

    def spotCountFromSpotImage(self, spot_image, threshhold,maxNum, sig):
        output={}
        output['positions'] = []
        output['threshholds'] = []
        isInLoop = True
        delta = round(4*sig)  # radius of area to splotch out after peak xy found
        deltaRange = range(-delta, delta)
        spot_image=np.pad(spot_image,delta)
        while isInLoop:
            xMaxIndex, yMaxIndex = np.unravel_index(spot_image.argmax(), spot_image.shape)
            maxValue = spot_image[xMaxIndex, yMaxIndex]
            print(maxValue)
            if len(output['positions'])>maxNum:
                isInLoop = False
            elif maxValue > threshhold:
                output['positions'].append([xMaxIndex-delta, yMaxIndex-delta])
                output['threshholds'].append(maxValue)
                spot_image[xMaxIndex + deltaRange, yMaxIndex + deltaRange] = 0
            else:
                isInLoop=False
        return output

class SpotCounter(iMicroscopyImageProcess):
    def __init__(self):
        self.spotLocationFinder=SpotCountLocations()
    def process(self, image, kernelSize=5, sig=3, threshhold=100,maxNum=500):
        spotCountData=self.spotLocationFinder.process(image, sig=sig, threshhold=threshhold,maxNum=maxNum)
        numSpotsPerChannel=[]
        for i in range(len(spotCountData)):
            numSpotsPerChannel.append(len(spotCountData[i]['positions']))
        return len(numSpotsPerChannel)



class CellDetectorCellMask(iMicroscopyImageProcess):
    def __init__(self, channels: list = [0, 0], diameter: float = 120,
                 model_type: str = 'cyto',BATCH_SIZE = 80,MINIMUM_CELL_AREA = 3000,default_flow_threshold =0.4):
        self.channels = channels
        self.diameter = diameter
        self.model_type = model_type  # options are 'cyto' or 'nuclei'
        self.default_flow_threshold = default_flow_threshold  # default is 0.4
        self.MINIMUM_CELL_AREA = MINIMUM_CELL_AREA
        self.BATCH_SIZE = BATCH_SIZE

    def process(self,image):
        model = models.Cellpose(gpu=1, model_type=self.model_type)
        masks = model.eval(image, batch_size=self.BATCH_SIZE, normalize=True,
                                        flow_threshold=self.default_flow_threshold, diameter=self.diameter,
                                        min_size=self.MINIMUM_CELL_AREA, channels=self.channels, progress=None)[0]
        masks=np.array(masks).tolist()
        return masks


class CellDetectorCellCount(iMicroscopyImageProcess):
    def __init__(self, channels: list = [0, 0], diameter: float = 120,
                 model_type: str = 'cyto',BATCH_SIZE = 80,MINIMUM_CELL_AREA = 3000):
        self.channels = channels
        self.diameter = diameter
        self.model_type = model_type  # options are 'cyto' or 'nuclei'
        self.default_flow_threshold = 0.4  # default is 0.4
        self.MINIMUM_CELL_AREA = 3000
        self.BATCH_SIZE = BATCH_SIZE

    def process(self,image):
        model = models.Cellpose(gpu=1, model_type=self.model_type)
        masks = model.eval(image, batch_size=self.BATCH_SIZE, normalize=True,
                                        flow_threshold=self.default_flow_threshold, diameter=self.diameter,
                                        min_size=self.MINIMUM_CELL_AREA, channels=self.channels, progress=None)[0]
        masks=np.array(masks)
        count=np.max(np.max(masks))
        return count



