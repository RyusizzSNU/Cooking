import numpy as np
import time

import torch
import torch.nn as nn
import torchvision.models as models
import torchvision.transforms as transforms
import cv2
from scipy.ndimage.filters import gaussian_filter
from torch.autograd import Variable
from PIL import Image
from PIL import ImageDraw

import detector
from cuboid import Cuboid3d
from cuboid_pnp_solver import CuboidPNPSolver

from pprint import pprint
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import seaborn as sns
from scipy.spatial.transform import Rotation as R

import json
import transformations

def find_object_poses(vertex2, aff, pnp_solver, config):
    '''Detect objects given network output'''

    # Detect objects from belief maps and affinities
    objects, all_peaks = find_objects(vertex2, aff, config)
    detected_objects = []
    obj_name = pnp_solver.object_name

#     print('find_object_poses: length of objects')
#     print(len(objects))
#     print('find_object_poses: objects')
#     pprint(objects)
#     print('find_object_poses: all_peaks')
#     print(all_peaks)

    for obj in objects:
        # Run PNP
        points = obj[1] + [(obj[0][0]*8, obj[0][1]*8)]
#         print('find_object_poses: points')
#         print(points)
        cuboid2d = np.copy(points)
        location, quaternion, projected_points = pnp_solver.solve_pnp(points)

        # Save results
        detected_objects.append({
            'name': obj_name,
            'location': location,
            'quaternion': quaternion,
            'cuboid2d': cuboid2d,
            'projected_points': projected_points,
            'score': obj[-1],
        })

    return detected_objects


def find_objects(vertex2, aff, config, numvertex=8):
    '''Detects objects given network belief maps and affinities, using heuristic method'''

    all_peaks = []
    peak_counter = 0
    for j in range(vertex2.size()[0]):
        belief = vertex2[j].clone()
        map_ori = belief.cpu().data.numpy()

        map = gaussian_filter(belief.cpu().data.numpy(), sigma=config.sigma)
        p = 1
        map_left = np.zeros(map.shape)
        map_left[p:,:] = map[:-p,:]
        map_right = np.zeros(map.shape)
        map_right[:-p,:] = map[p:,:]
        map_up = np.zeros(map.shape)
        map_up[:,p:] = map[:,:-p]
        map_down = np.zeros(map.shape)
        map_down[:,:-p] = map[:,p:]

        peaks_binary = np.logical_and.reduce(
                            (
                                map >= map_left, 
                                map >= map_right, 
                                map >= map_up, 
                                map >= map_down, 
                                map > config.thresh_map)
                            )
        peaks = zip(np.nonzero(peaks_binary)[1], np.nonzero(peaks_binary)[0]) 
        

        # Computing the weigthed average for localizing the peaks
        peaks = list(peaks)
        win = 5
        ran = win // 2
        peaks_avg = []
        for p_value in range(len(peaks)):
            p = peaks[p_value]
            weights = np.zeros((win,win))
            i_values = np.zeros((win,win))
            j_values = np.zeros((win,win))
            for i in range(-ran,ran+1):
                for j in range(-ran,ran+1):
                    if p[1]+i < 0 \
                            or p[1]+i >= map_ori.shape[0] \
                            or p[0]+j < 0 \
                            or p[0]+j >= map_ori.shape[1]:
                        continue 

                    i_values[j+ran, i+ran] = p[1] + i
                    j_values[j+ran, i+ran] = p[0] + j

                    weights[j+ran, i+ran] = (map_ori[p[1]+i, p[0]+j])

            # if the weights are all zeros
            # then add the none continuous points
            OFFSET_DUE_TO_UPSAMPLING = 0.4395
            try:
                peaks_avg.append(
                    (np.average(j_values, weights=weights) + OFFSET_DUE_TO_UPSAMPLING, \
                     np.average(i_values, weights=weights) + OFFSET_DUE_TO_UPSAMPLING))
            except:
                peaks_avg.append((p[0] + OFFSET_DUE_TO_UPSAMPLING, p[1] + OFFSET_DUE_TO_UPSAMPLING))
        # Note: Python3 doesn't support len for zip object
        peaks_len = min(len(np.nonzero(peaks_binary)[1]), len(np.nonzero(peaks_binary)[0]))

        peaks_with_score = [peaks_avg[x_] + (map_ori[peaks[x_][1],peaks[x_][0]],) for x_ in range(len(peaks))]

        id = range(peak_counter, peak_counter + peaks_len)

        peaks_with_score_and_id = [peaks_with_score[i] + (id[i],) for i in range(len(id))]

        all_peaks.append(peaks_with_score_and_id)
        peak_counter += peaks_len

    objects = []

    # Check object centroid and build the objects if the centroid is found
    for nb_object in range(len(all_peaks[-1])):
        if all_peaks[-1][nb_object][2] > config.thresh_points:
            objects.append([
                [all_peaks[-1][nb_object][:2][0],all_peaks[-1][nb_object][:2][1]],
                [None for i in range(numvertex)],
                [None for i in range(numvertex)],
                all_peaks[-1][nb_object][2]
            ])
            
#     pprint(objects)
            
    # Working with an output that only has belief maps
    if aff is None:
        if len (objects) > 0 and len(all_peaks)>0 and len(all_peaks[0])>0:
            for i_points in range(8):
                if  len(all_peaks[i_points])>0 and all_peaks[i_points][0][2] > config.threshold:
                    objects[0][1][i_points] = (all_peaks[i_points][0][0], all_peaks[i_points][0][1])
    else:
        # For all points found
        for i_lists in range(len(all_peaks[:-1])):
            lists = all_peaks[i_lists]

            for candidate in lists:
                if candidate[2] < config.thresh_points:
                    continue

                i_best = -1
                best_dist = 10000 
                best_angle = 100
                for i_obj in range(len(objects)):
                    center = [objects[i_obj][0][0], objects[i_obj][0][1]]

                    # integer is used to look into the affinity map, 
                    # but the float version is used to run 
                    point_int = [int(candidate[0]), int(candidate[1])]
                    point = [candidate[0], candidate[1]]

                    # look at the distance to the vector field.
                    v_aff = np.array([
                                    aff[i_lists*2, 
                                    point_int[1],
                                    point_int[0]].data.item(),
                                    aff[i_lists*2+1, 
                                        point_int[1], 
                                        point_int[0]].data.item()]) * 10

                    # normalize the vector
                    xvec = v_aff[0]
                    yvec = v_aff[1]

                    norms = np.sqrt(xvec * xvec + yvec * yvec)

                    xvec/=norms
                    yvec/=norms

                    v_aff = np.concatenate([[xvec],[yvec]])

                    v_center = np.array(center) - np.array(point)
                    xvec = v_center[0]
                    yvec = v_center[1]

                    norms = np.sqrt(xvec * xvec + yvec * yvec)

                    xvec /= norms
                    yvec /= norms

                    v_center = np.concatenate([[xvec],[yvec]])

                    # vector affinity
                    dist_angle = np.linalg.norm(v_center - v_aff)

                    # distance between vertexes
                    dist_point = np.linalg.norm(np.array(point) - np.array(center))

                    if dist_angle < config.thresh_angle and (best_dist > 1000 or best_dist > dist_point):
                        i_best = i_obj
                        best_angle = dist_angle
                        best_dist = dist_point

                if i_best == -1:
                    continue

                if objects[i_best][1][i_lists] is None \
                        or best_angle < config.thresh_angle \
                        and best_dist < objects[i_best][2][i_lists][1]:
                    objects[i_best][1][i_lists] = ((candidate[0])*8, (candidate[1])*8)
                    objects[i_best][2][i_lists] = (best_angle, best_dist)
#                 else:
#                     print('objects[i_best][1][i_lists] is None: ' + str(objects[i_best][1][i_lists] is None))
#                     print('best_angle < config.thresh_angle: ' + str(best_angle < config.thresh_angle))
#                     print('best_dist < objects[i_best][2][i_lists][1]: ' + str(best_dist < objects[i_best][2][i_lists][1]))
#                     print('best_dist: ' + str(best_dist))
#                     print('objects[i_best][2][i_lists][1]: ' + str(objects[i_best][2][i_lists][1]))
#                     print('objects[i_best][1][i_lists]: ' + str(objects[i_best][2][i_lists]))

#     pprint(objects)

    return objects, all_peaks


class Draw(object):
    """Drawing helper class to visualize the neural network output"""

    def __init__(self, im):
        """
        :param im: The image to draw in.
        """
        self.draw = ImageDraw.Draw(im)

    def draw_line(self, point1, point2, line_color, line_width=2):
        """Draws line on image"""
        if point1 is not None and point2 is not None:
            self.draw.line([point1, point2], fill=line_color, width=line_width)

    def draw_dot(self, point, point_color, point_radius):
        """Draws dot (filled circle) on image"""
        if point is not None:
            xy = [
                point[0] - point_radius,
                point[1] - point_radius,
                point[0] + point_radius,
                point[1] + point_radius
            ]
            self.draw.ellipse(xy,
                              fill=point_color,
                              outline=point_color
                              )

    def draw_cube(self, points, color=(255, 0, 0)):
        """
        Draws cube with a thick solid line across
        the front top edge and an X on the top face.
        """

        # draw front
        self.draw_line(points[0], points[1], color)
        self.draw_line(points[1], points[2], color)
        self.draw_line(points[3], points[2], color)
        self.draw_line(points[3], points[0], color)

        # draw back
        self.draw_line(points[4], points[5], color)
        self.draw_line(points[6], points[5], color)
        self.draw_line(points[6], points[7], color)
        self.draw_line(points[4], points[7], color)

        # draw sides
        self.draw_line(points[0], points[4], color)
        self.draw_line(points[7], points[3], color)
        self.draw_line(points[5], points[1], color)
        self.draw_line(points[2], points[6], color)

        # draw dots
        self.draw_dot(points[0], point_color=color, point_radius=4)
        self.draw_dot(points[1], point_color=color, point_radius=4)

        # draw x on the top
        self.draw_line(points[0], points[5], color)
        self.draw_line(points[1], points[4], color)


        
        
# ============================== SETTINGS ==============================
weight_path = '../../../scripts/train_div_no_crop/net_epoch_10.pth'
# weight_path = '../../../scripts/train_turtle_no_crop/net_epoch_10.pth'
# weight_path = '../../../scripts/train_turtle_no_crop_2000/net_epoch_10.pth'
# weight_path = '../../../scripts/train_turtle_no_crop_1800/net_epoch_10.pth'
# infer_save_path = 'turtle_checking_output_1800_test'
# infer_save_path = 'turtle_temp'
infer_save_path = 'div_temp'
sample_image_paths = []
sample_gt_paths = []
for i in range(0, 100, 1):
# for i in range(1800, 2000, 1):
    # 입력 이미지 추가
#     sample_image_paths.append('../../../fat_div/003_cracker_box_16k/kitchen_0/' + str(i).zfill(6) + '.left.jpg')
    sample_image_paths.append('../../../fat_div/004_sugar_box_16k/kitchen_1/' + str(i).zfill(6) + '.left.jpg')
    sample_image_paths.append('../../../fat_div/005_tomato_soup_can_16k/kitchen_2/' + str(i).zfill(6) + '.left.jpg')
#     sample_image_paths.append('../../../fat_div/006_mustard_bottle_16k/kitchen_3/' + str(i).zfill(6) + '.left.jpg')
#     sample_image_paths.append('../../../fat_div/007_tuna_fish_can_16k/kitchen_4/' + str(i).zfill(6) + '.left.jpg')
#     sample_image_paths.append('../../../fat_div/008_pudding_box_16k/kitedemo_0/' + str(i).zfill(6) + '.left.jpg')
#     sample_image_paths.append('../../../fat_div/009_gelatin_box_16k/kitedemo_1/' + str(i).zfill(6) + '.left.jpg')
#     sample_image_paths.append('../../../fat_div/010_potted_meat_can_16k/kitedemo_2/' + str(i).zfill(6) + '.left.jpg')
#     sample_image_paths.append('../../../fat_div/011_banana_16k/kitedemo_3/' + str(i).zfill(6) + '.left.jpg')
#     sample_image_paths.append('../../../fat_div/019_pitcher_base_16k/kitedemo_4/' + str(i).zfill(6) + '.left.jpg')
#     sample_image_paths.append('../../../fat_div/021_bleach_cleanser_16k/temple_0/' + str(i).zfill(6) + '.left.jpg')
#     sample_image_paths.append('../../../fat_div/024_bowl_16k/temple_1/' + str(i).zfill(6) + '.left.jpg')
#     sample_image_paths.append('../../../fat_div/025_mug_16k/temple_2/' + str(i).zfill(6) + '.left.jpg')
#     sample_image_paths.append('../../../fat_div/035_power_drill_16k/temple_3/' + str(i).zfill(6) + '.left.jpg')
#     sample_image_paths.append('../../../fat_div/036_wood_block_16k/temple_4/' + str(i).zfill(6) + '.left.jpg')
    
#     sample_image_paths.append('../../../turtlebot_normal/' + str(i).zfill(6) + '.png')
#     sample_image_paths.append('../../../turtlebot_2000/' + str(i).zfill(6) + '.png')
#     sample_image_paths.append('../../../turtlebot_test_200/' + str(i).zfill(6) + '.png')

    # 입력 이미지에 대한 정답 추가 (정확도 계산이 필요한 경우)
#     sample_gt_paths.append('../../../turtlebot_2000/' + str(i).zfill(6) + '.json')
#     sample_gt_paths.append('../../../turtlebot_test_200/' + str(i).zfill(6) + '.json')
    sample_gt_paths.append('../../../fat_div/004_sugar_box_16k/kitchen_1/' + str(i).zfill(6) + '.left.json')
    sample_gt_paths.append('../../../fat_div/005_tomato_soup_can_16k/kitchen_2/' + str(i).zfill(6) + '.left.json')

# 사이즈 정보 (cm), -> 학습데이터의 object_settings.json에 있는 정보.
# cuboid_dimension = np.array([35.180698394775391, 37.58489990234375, 27.55579948425293]) # 터틀봇
cuboid_dimension1 = np.array([9.2677001953125, 17.625299453735352, 4.5134000778198242]) # 004_sugar_box
cuboid_dimension2 = np.array([6.7659001350402832, 10.185500144958496, 6.771399974822998]) # 005_tomato_soup_can

# 각도변환 정보 (사용안함)
# 터틀봇
# fixed_model_transformation = np.array([
#     [ -0.002899999963119626, -0.00079999997979030013, 2, 0 ],
#     [ -2, 0.00039999998989515007, -0.002899999963119626, 0 ],
#     [ -0.00039999998989515007, -2, -0.00079999997979030013, 0 ],
#     [ 30.180099487304688, -7.612800121307373, 40.262599945068359, 1 ]
# ])

# tresholds
config_detect = lambda: None
config_detect.mask_edges = 1
config_detect.mask_faces = 1
config_detect.vertex = 1
config_detect.threshold = 0.5 # 기존 0.5 -> 사용안함
config_detect.softmax = 1000
config_detect.thresh_angle = 0.5 # 기존 0.5
config_detect.thresh_map = 0.01 # 기존 0.01
config_detect.sigma = 3
config_detect.thresh_points = 0.03 # 기존 0.1

transform = transforms.Compose([
    # transforms.Scale(IMAGE_SIZE),
    # transforms.CenterCrop((imagesize,imagesize)),
    transforms.ToTensor(),
    transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5)),
    ])

net = detector.DopeNetwork()
net = torch.nn.DataParallel(net, [0]).cuda()
net.load_state_dict(torch.load(weight_path))
net.eval()

# FAT용
camera_matrix = np.array([
                [768.16, 0.0, 480.0],
                [0.0, 768.16, 270.0],
                [0.0, 0.0, 1.0]
            ])

# 터틀봇용
# camera_matrix = np.array([
#                 [256.0, 0.0, 256.0],
#                 [0.0, 256.0, 256.0],
#                 [0.0, 0.0, 1.0]
#             ])

# pnp_solver = CuboidPNPSolver(
#                     0,
#                     cuboid3d=Cuboid3d(cuboid_dimension)
#                 )

pnp_solver1 = CuboidPNPSolver(
                    'sugar_box',
                    cuboid3d=Cuboid3d(cuboid_dimension1)
                )
pnp_solver2 = CuboidPNPSolver(
                    'tomato_soup_can',
                    cuboid3d=Cuboid3d(cuboid_dimension2)
                )

dist_coeffs = np.zeros((4, 1))
# pnp_solver.set_camera_intrinsic_matrix(camera_matrix)
# pnp_solver.set_dist_coeffs(dist_coeffs)

pnp_solver1.set_camera_intrinsic_matrix(camera_matrix)
pnp_solver1.set_dist_coeffs(dist_coeffs)
pnp_solver2.set_camera_intrinsic_matrix(camera_matrix)
pnp_solver2.set_dist_coeffs(dist_coeffs)
    
plt.figure(figsize=(12, 6.7))




# ============================== INFERENCE ==============================
location_errors = []
location_gt_z = []
e_errors = []
for image_idx, sample_image_path in enumerate(sample_image_paths):
    print('======================IMAGE %d======================' % (image_idx + 1))
    sample_image = cv2.imread(sample_image_path)
    sample_image_tensor = transform(sample_image)
    sample_image_torch = Variable(sample_image_tensor).cuda().unsqueeze(0)

    sample_image_copy = sample_image.copy()
    im = Image.fromarray(sample_image_copy)

    out, seg = net(sample_image_torch)
    vertex2 = out[-1][0]
    aff = seg[-1][0]
    
#     print('vertex2 size')
#     print(vertex2.size())
#     exit(1)
    
    # save heatmap for each vertices
#     for vertex_idx in range(9):
#         vt2 = vertex2[vertex_idx].detach().cpu()
#         sns.heatmap(vt2, cmap='viridis')
#         plt.savefig('vertex_heatmap' + str(image_idx + 1).zfill(2) + '_' + str(vertex_idx + 1) + '.png', dpi=300)
#         plt.clf()
#     print('vertex heatmap saved')
    
    # Find objects from network output
#     results = find_object_poses(vertex2, aff, pnp_solver, config_detect)
    if image_idx % 2 == 0:
        results = find_object_poses(vertex2, aff, pnp_solver1, config_detect)
    elif image_idx % 2 == 1:
        results = find_object_poses(vertex2, aff, pnp_solver2, config_detect)

    print('detected objects: %d' % len(results))
    
    
    
    
            
    
    # 위치 정확도 계산
    # 한 이미지에 대한 추론 정보가 1개일때만 정확도 계산에 포함 (올바로 추론한것만 정확도 계산에 포함한다.)
    if len(results) == 1:
        for i_r, result in enumerate(results):
            if result['location'] is None:
                continue
            location_infer = result['location']

            with open(sample_gt_paths[image_idx]) as gt_file:
                gt_data = json.load(gt_file)
                location_gt = np.array(gt_data['objects'][0]['location'])

                location_error = abs(location_infer - location_gt)
                print('location error:')
                print(location_error)
                gt_z = location_gt[2]
                
                location_errors.append(location_error)
                location_gt_z.append(gt_z)
                
            
    # 각도 정확도 계산
    # 한 이미지에 대한 추론 정보가 1개일때만 정확도 계산에 포함 (올바로 추론한것만 정확도 계산에 포함한다.)
    if len(results) == 1:
        for i_r, result in enumerate(results):
            if result["quaternion"] is None:
                continue
            ori_infer = result["quaternion"]

            # transform orientation
#             model_transform = transformations.quaternion_from_matrix(fixed_model_transformation)
#             transformed_ori = transformations.quaternion_multiply(ori, model_transform)
            
            with open(sample_gt_paths[image_idx]) as gt_file:
                gt_data = json.load(gt_file)
                ori_gt = np.array(gt_data['objects'][0]['quaternion_xyzw'])
            
            # 추론 quaternion -> euler 변환
            ori_infer_x = ori_infer[0]
            ori_infer_y = ori_infer[1]
            ori_infer_z = ori_infer[2]
            ori_infer_w = ori_infer[3]
            r_infer = R.from_quat([ori_infer_x, ori_infer_y, ori_infer_z, ori_infer_w])
            e_infer = r_infer.as_euler('yzx', degrees=True)
            e_infer_y = e_infer[0]
            
            # gt quaternion -> euler 변환
            ori_gt_x = ori_gt[0]
            ori_gt_y = ori_gt[1]
            ori_gt_z = ori_gt[2]
            ori_gt_w = ori_gt[3]
            r_gt = R.from_quat([ori_gt_x, ori_gt_y, ori_gt_z, ori_gt_w])
            e_gt = r_gt.as_euler('yzx', degrees=True)
            e_gt_y = e_gt[0]
            
            e_error = abs(e_gt_y - e_infer_y)
            if e_error > 180:
                e_error = 360 - e_error
            print('euler_y error:')
            print(e_error)
            e_errors.append(e_error)
            
            print(ori_infer)
            print(ori_gt)
    

    # Draw the cube
    draw = Draw(im)

    for i_r, result in enumerate(results):
        if result["location"] is None:
            continue
        if None not in result['projected_points']:
            print('============OBJECT %d============' % (i_r + 1))
#             print('projected points')
#             print(result['projected_points'])
            points2d = []
            for pair in result['projected_points']:
                points2d.append(tuple(pair))
    #         print(points2d)
            draw.draw_cube(points2d)
    #         print('drawed')

    im.save(infer_save_path + '/' + str(image_idx + 1).zfill(3) + '.png')
#     print('cube drawing saved')
    
    with open(infer_save_path + '/results' + str(image_idx + 1).zfill(3) + '.txt', 'w') as f:
        print(results, file=f)
#     print('output results saved')
    
    # (z, x, pitch) -> (x, y, theta)
    if len(results) == 0 or (results[0]['location'] is None):
        continue
    pose_x = results[0]['location'][2]
    pose_y = results[0]['location'][0]
    q_x = results[0]['quaternion'][0]
    q_y = results[0]['quaternion'][1]
    q_z = results[0]['quaternion'][2]
    q_w = results[0]['quaternion'][3]
    r = R.from_quat([q_x, q_y, q_z, q_w])
    e = r.as_euler('yzx', degrees=True)
    e_y = e[0]
    pose = [pose_x, pose_y, e_y]
    
    with open(infer_save_path + '/pose' + str(image_idx + 1).zfill(3) + '.txt', 'w') as f:
        f.write(str(pose[0]))
        f.write(',')
        f.write(str(pose[1]))
        f.write(',')
        f.write(str(pose[2]))
#     print('translated pose saved')

# 평균 location 오차 (각 x, y, z 축에 대하여, cm)
print('mean location error:')
print(np.mean(location_errors, axis=0))
print('mean of location z:')
print(np.mean(location_gt_z))

# 평균 euler 각도 오차 (y성분)
print('mean euler_y error:')
print(np.mean(e_errors))
