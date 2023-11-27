import numpy as np
import matplotlib.pyplot as plt
import os
import cv2
import imageio
from tqdm import tqdm
import threading as th

path = '/externalSSD/htn_experiment/trial2'

def progressBar(count_value, total, id, suffix=''):
    bar_length = 100
    filled_up_Length = int(round(bar_length* count_value / float(total)))
    percentage = round(100.0 * count_value/float(total),1)
    bar = '=' * filled_up_Length + '-' * (bar_length - filled_up_Length)
    print('%s -- [%s] %s%s ...%s\r' %(id, bar, percentage, '%', suffix))
    

def conver_slice(list, threadID=0):
    global path
    for item, i in zip(sorted(list), range(len(list))):
        cvimage = np.load(os.path.join(path, item))
        cv2.imwrite(os.path.join('/externalSSD/htn_experiment/trial2', item + '.png'), cvimage)
        progressBar(i,len(list), threadID)

# n_samples = 100
# x = np.arange(0, n_samples, 1)
# x = x - n_samples/2
# # total_time = 5

# avg_vel = total_time / n_samples

# y = np.ones((n_samples, 1)) * avg_vel

# list = []
# third = int(n_samples/5)
# for i in range(third):
#     list.append((1.5*avg_vel)-(i * avg_vel / (2*third)))
# for i in range(3*third):
#     list.append(avg_vel)
# for i in range(third):
#     list.append(avg_vel + (i * avg_vel / (2*third)))

# listnp = np.array(list)
# listnpprime = np.diff(listnp)
# fig = plt.figure()
# plt.plot(x, y)
# plt.plot(x, list)
# plt.plot(x[:-1], listnpprime)
# # plt.show()

# func = np.asarray([1/(xi - 2) for xi in x])
# # func2 = np.asarray([2*xi for xi in x])
# func3 = np.asarray([(y1-y0)/(x1-x0) for x0, x1, y0, y1 in zip(x, x[1:], func, func[1:])])

# funcprime = np.diff(func)

# plt.plot(x, func)
# # plt.plot(x, func2)
# plt.plot(x[:-1], func3)
# plt.plot(x[:-1], funcprime)
# plt.show()

ls = sorted(os.listdir(path))
ls = ls[:-1]
print(ls)
print(ls[-1])
print(ls[0])
total_time = (int(ls[-1][:-4]) - int(ls[0][:-4]))/1e9
print(f'Total time: {total_time} seconds')
freq = len(ls)/total_time
print(f'Frequency: {freq} Hz')

prev_gray = None

diff_list = []

# for item in sorted(ls):
#     cvimage = np.load(os.path.join('/externalSSD/htn_experiment/0/Zed2', item))
#     # print(cvimage.shape)
#     cv2.imshow('image', cvimage)
#     cv2.waitKey(1)

    # while cv2.waitKey(1) & 0xFF != ord('q'):
    #     pass

#     gray = cv2.cvtColor(cvimage, cv2.COLOR_BGR2GRAY)
#     if prev_gray is None:
#         prev_gray = gray

#     diff_frame = cv2.absdiff(prev_gray, gray)
#     diff = np.sum(diff_frame)
#     diff_list.append(diff)

# plt.figure()
# plt.plot(diff_list)
# plt.show()

# errs = []
# for i in range(len(diff_list)):
#     if diff_list[i] > 1.5*(sum(diff_list)/len(diff_list)):
#         print(i)
#         errs.append(i)
# print(f'Number of errors: {len(errs)}')
# for i in errs:
#     cvimage = np.load(os.path.join('/externalSSD/htn_experiment/0/Zed', ls[i]))
#     cv2.imshow('image', cvimage)
#     while cv2.waitKey(1) & 0xFF != ord('q'):
#         pass

slices = []
n_slices = 16
slice_dim = int(len(ls)/n_slices)
for i in range(n_slices):
    slices.append(ls[i*slice_dim:(i+1)*slice_dim])

print(slices)

threads = []
for i in range(n_slices):
    t = th.Thread(target=conver_slice, args=(slices[i],i))
    threads.append(t)
    t.start()

for t in threads:
    t.join()


    # cv2.imshow('image', cvimage)
    # while cv2.waitKey(1) & 0xFF != ord('q'):
    #     pass