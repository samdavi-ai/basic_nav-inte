"""Private guard using separating axes of the padded body and occupied cells."""
import math
import numpy as np
import rclpy
from generation_guard import GenerationGuard


def cell_intersects(centers, pose, half, res, travel):
    """Touch is occupied. Travel expands every projection by a metric motion bound."""
    x,y,yaw = pose
    c,s = math.cos(yaw),math.sin(yaw)
    a,b = half
    offsets = centers - (x,y)
    axes = np.array([[1,0],[0,1],[c,s],[-s,c]])
    h=res/2
    extents=np.array([abs(c)*a+abs(s)*b+h,abs(s)*a+abs(c)*b+h,
                      a+h*(abs(c)+abs(s)),b+h*(abs(c)+abs(s))])
    return np.any(np.all(np.abs(offsets @ axes.T) <= extents+travel+1e-12,axis=1))


class RectangularGuard(GenerationGuard):
    def on_footprint(self,msg):
        super().on_footprint(msg)
        if self.half is None:
            return
        points=np.array([[p.x,p.y] for p in msg.polygon.points])
        if len(points)!=4 or not np.allclose(np.sort(np.linalg.norm(points-np.roll(points,1,axis=0),axis=1)),
                                              [.70,.70,.72,.72],atol=.0001):
            self.half=None
            self.plan_valid=False

    @staticmethod
    def fits(x,y,yaw,origin,end,margin):
        c,s=abs(math.cos(yaw)),abs(math.sin(yaw))
        extent=np.array([c*.36+s*.35,s*.36+c*.35])+margin
        return not (np.any([x,y]-extent<origin) or np.any([x,y]+extent>end))

    def clear(self,x,y,heading,margin):
        return (self.fits(x,y,heading,self.origin,self.extent,margin) and
                not cell_intersects(self.obstacles,(x,y,heading),(.36,.35),self.res,margin))

    def validate_plan(self):
        self.plan_valid=False
        if self.half is None or self.plan is None or self.global_grid is None:
            return
        tree,origin,end,res=self.global_grid
        tangent=self.plan.copy()
        delta=np.diff(tangent[:,:2],axis=0)
        tangent[:-1,2]=np.where(np.linalg.norm(delta,axis=1)>1e-7,
                                  np.arctan2(delta[:,1],delta[:,0]),tangent[:-1,2])
        travel=res/4
        for path in (self.plan,tangent):
            poses=self.interpolate(path,self.half,res/2)
            neighbors=tree.query_ball_point(poses[:,:2],math.hypot(.36,.35)+res/math.sqrt(2)+travel)
            for pose,indices in zip(poses,neighbors):
                if not self.fits(*pose,origin,end,travel):
                    return
                if indices and cell_intersects(tree.data[indices],pose,(.36,.35),res,travel):
                    return
        self.plan_valid=True


if __name__=='__main__':
    rclpy.init()
    node=RectangularGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
