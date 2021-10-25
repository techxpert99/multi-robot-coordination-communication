from controller.wrappers import ThreadWrapper
import numpy as np
import cv2

class Visualizer:

    def __init__(self,store):
        self._store = store
        store.set('visualizer','callbacks',[])
        self._wait = round(1000*store.get('general','visualizer_node_callback_interval'))
        self._tw = ThreadWrapper(self._callback,0)
        self._tw.__thread__.setName("visualizer-thread")
        self.add_visualization_callback(('Map',self.critical_map_matrix_function))
        self._tw.start()

    def _callback(self):
        self._store.lock('visualizer')
        callbacks = list(self._store.unsafe_get('visualizer','callbacks'))
        self._store.release('visualizer')
        for window_name,callback in callbacks:
            matrix = callback()
            if matrix is not None:
                cv2.imshow(window_name,matrix)
        cv2.waitKey(self._wait)
    
    def destroy(self):
        self._tw.stop()
        cv2.destroyAllWindows()

    def cv2_tf(self,point):
        return point[1],point[0]

    def rasterize_matrix(self,matrix,color_function):
        row,col = matrix.shape
        image = np.zeros((row,col,3),np.uint8)
        for i in range(row):
            for j in range(col):
                image[i,j] = color_function(matrix[i,j])
        return image
    
    def draw_path(self,image,path,color=(0,0,0),thickness=1):
        for k in range(1,len(path)):
            image = cv2.line(image,self.cv2_tf(path[k-1]),self.cv2_tf(path[k]),color,thickness)
        return image

    def draw_point(self,image,point,color=(0,0,0),radius=1):
        return cv2.circle(image,self.cv2_tf(point),radius,color,cv2.FILLED)
    
    def add_visualization_callback(self,matrix_function):
        self._store.lock('visualizer')
        self._store.unsafe_get('visualizer','callbacks').append(matrix_function)
        self._store.release('visualizer')

    def critical_map_matrix_function(self):

        if self._store.get('mapper','cmap').get_critical_map() is None: return None

        cx,cy,ct = self._store.get('estimator','position')

        self._store.lock('mapper')
        cmap = self._store.unsafe_get('mapper','cmap')
        matrix = cmap.get_critical_map()

        def col_fn(x):
            if x == 0: return (0,0,0)
            elif x == 1: return (0,128,255)
            elif x == 2: return (255,255,0)
            elif x == 3: return (255,255,255)
            return (128,128,128)

        current_cell = cmap.itf(cx,cy)
        image = self.rasterize_matrix(matrix,col_fn)
        self._store.release('mapper')

        image = self.draw_point(image,current_cell,(0,0,255))

        if self._store.has('planner','plan'):
            plan,plan_tf = self._store.get('planner','plan')
            if plan:
                image = self.draw_path(image,plan,(255,0,0))
                image = self.draw_point(image,plan[0],(0,255,255))
                image = self.draw_point(image,plan[-1],(0,255,0))
        
        image = cv2.resize(image,(800,800))
        return image
    