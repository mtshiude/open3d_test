import numpy as np
import open3d as o3d

from PIL import Image
import matplotlib.pyplot as plt
import time
class point_control(object):
    def __init__(self):
        self.easy_test()
        input('done')


        print(self.make_point_group())
        np_pcd = self.make_point_group()
        pcd = self.transrate_pcd(np_pcd)

        #pcd = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, 0.03) # ボクセルのサイズ指定
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
        #im = o3d.visualization.draw_geometries([pcd,mesh],point_show_normal=True)
        #plt.imshow(np.asarray(im))
        #plt.show()

    def make_point_group(self):
        #球状の点群を生成
        #立方体中にランダムな点を作ってL2ノルムで正規化
        sphere = np.random.rand(10000, 3) - np.array([0.5, 0.5, 0.5])
         
        sphere = sphere/np.linalg.norm(sphere, axis=1, keepdims=True)
        return sphere
    
    def transrate_pcd(self,data):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(data)
        return pcd
    
    def easy_test(self):
        #create sphere triangle mesh
        #mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
        mesh = o3d.geometry.TriangleMesh.create_sphere()

        mesh.compute_vertex_normals()
        #o3d.visualization.draw_geometries([mesh])
        #sampling point cloud from sphere triangle mesh
        pcd = mesh.sample_points_uniformly(number_of_points=1000)
        #input('r')
        #self.custom_draw_geometry_with_rotation(mesh)
        self.custom_draw_geometry_with_custom_fov(mesh,np.pi*2)
        #o3d.visualization.draw_geometries([pcd])
    
    def custom_draw_geometry_with_custom_fov(self,pcd, rotate_deg):
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(pcd)
        ctr = vis.get_view_control()
        ctr.rotate(rotate_deg,0)
        print("Field of view %.2f" % ctr.get_field_of_view())
        print("rotate %.2f" % rotate_deg)
        vis.run()
        vis.capture_depth_image('capture_rgbd/'+str(rotate_deg)+'_depth.png')
        vis.capture_screen_image('capture_rgbd/' + str(rotate_deg) + '_image.png')
        vis.destroy_window()

    def custom_draw_geometry_with_rotation(self,pcd,rotate_deg=30):
        self.all_deg = 0


        o3d.visualization.draw_geometries_with_animation_callback([pcd],
                                                                  self.rotate_view)
    def rotate_view(self,vis):
        ctr = vis.get_view_control()
        ctr.change_field_of_view(step=60)

        vis.capture_depth_image('capture_rgbd/' + str(self.all_deg) + '_depth.png')
        vis.capture_screen_image('capture_rgbd/' + str(self.all_deg) + '_image.png')

        ctr.rotate(60,0)
        print(self.all_deg)
        if self.all_deg>360:
            vis.destroy_window()
        else:
            self.all_deg += 60
        return False




    
if __name__ == "__main__":
    pc = point_control()
'''
print(np.asarray(pcd.points))
print(np.asarray(pcd.points).shape)
down_pcd = pcd.voxel_down_sample(voxel_size=0.05)
print(np.asarray(pcd.points))
print(np.asarray(pcd.points).shape)
#input()
#down_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
im = o3d.visualization.draw_geometries([down_pcd],point_show_normal=True)

plt.imshow(np.asarray(im))
plt.show()
#print("Print a normal vector of the 0th point")
#print(down_pcd.normals[0])
'''
