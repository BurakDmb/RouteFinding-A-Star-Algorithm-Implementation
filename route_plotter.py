
import matplotlib.pyplot as plt

from scipy import ndimage

def plotGraph(things, obstacles, size= 300, path = None):  # pragma: no cover
        image = plt.imread("robot.png")
    
        for obstacle in obstacles:
            x,y,r = obstacle.x, obstacle.y, obstacle.radius
            plt.plot(x, y, "o", ms=3*r, color='#FFA500')


        for thing in things:
            x, y, theta = thing.x, thing.y, thing.angle
        
            xlim = 20
            ylim = 20
            
            if(math.degrees(theta)<0):
                plt.imshow((ndimage.rotate(image, (-90+math.degrees(theta))) * 255).astype(np.uint8), extent=[x-1*xlim, x+1*xlim, y-0.75*ylim, y+0.75*ylim], zorder=2)
            else:
                plt.imshow((ndimage.rotate(image, (-90+math.degrees(theta))) * 255).astype(np.uint8), extent=[x-1*xlim, x+1*xlim, y-0.75*ylim, y+0.75*ylim], zorder=2)
            
        if path:
            plt.plot(path[0], path[1], zorder=3)
        plt.xlim(0, 200)
        plt.ylim(0, 200)

        plt.show()
        