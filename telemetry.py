import subprocess as sp
import shlex
import math
import mapnik
import numpy as np
from datetime import datetime
import scipy.ndimage as nd
from scipy.signal import savgol_filter
from haversine import haversine
from scipy.interpolate import interp1d, CubicSpline
import georaster as gr
import time
import sys
import struct

coeff_haver = 6.4e6 * math.pi / 180.
def fast_haversine(initial, final, unit):
    retval = ((final[1] - initial[1])/2)**2 + (final[0] - initial[0])**2
    return retval * coeff_haver

def to_mercator(proj, lonlat):
    xy = proj.forward(mapnik.Coord(lonlat[1], lonlat[0]))
    return np.array((xy.x, xy.y))

def read_int(stream, raise_if_empty=True):
    retval = stream.read(4)
    if not retval:
        if raise_if_empty:
            raise Exception(f'truncated file : expected 4 bytes, got {len(retval)}')
        else:
            return None
    if len(retval) < 4:
        raise Exception(f'truncated file : expected 4 bytes, got {len(retval)}')
    return int.from_bytes(retval, sys.byteorder)

def read_float32(stream, raise_if_empty=True):
    retval = stream.read(4)
    if not retval:
        if raise_if_empty:
            raise Exception(f'truncated file : expected 4 bytes, got {len(retval)}')
        else:
            return None
    if len(retval) < 4:
        raise Exception(f'truncated file : expected 4 bytes, got {len(retval)}')
    return struct.unpack('f', retval)

dtype_face = np.dtype([('id', np.uint32), ('face', np.float32, 22)])

def read_buffer(stream, per_element, raise_if_empty=True):
    size = read_int(stream, raise_if_empty)
    if size is None:
        return None
    buffer = stream.read(size * per_element)
    if len(buffer) < size * per_element:
        raise Exception(f'truncated file : expected {size}x{per_element} bytes, got {len(buffer)}')
    return buffer

def read_array(stream, dtype, width, raise_if_empty=True):
    buffer = read_buffer(stream, width * np.dtype(dtype).itemsize, raise_if_empty)
    if buffer is None:
        return None
    retval = np.frombuffer(buffer, dtype=dtype)
    if width != 1:
        return retval.reshape((retval.shape[0] // width, width))
    else:
        return retval

def read_inertia(stream, raise_if_empty=True):
    return read_array(stream, np.float32, 3, raise_if_empty)

def read_gps5(stream, raise_if_empty=True):
    return read_array(stream, np.float64, 5, raise_if_empty)

def read_faces(stream, raise_if_empty=True):
    size = read_int(stream, raise_if_empty)
    if size is None:
        return None
    retval = []
    for i in range(size):
        faces = read_array(stream, dtype_face, 1, raise_if_empty)
        if faces is not None:
            retval.append(faces)
    return retval

def read_timestamp(stream, raise_if_empty=True):
    buffer = read_buffer(stream, 1, raise_if_empty)
    if len(buffer) != 16:
        if raise_if_empty:
            raise Exception(f'truncated file : expected 16 bytes, got {len(buffer)}')
        else:
            return None
    return buffer.decode()

class Decode(object):
    def __init__(self, telemetry_bin, elevation_tiff, srs, fps, count):
        self.fps_rounded = round(fps)
        self.fps = fps
        self.telemetry_bin = telemetry_bin
#EPSG3035
        EPSG_4326 = '+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs'
        self.proj_lonlat = mapnik.Projection(EPSG_4326)
        self.proj_dest = mapnik.Projection(srs)
        self.transform_lonlat_osm = mapnik.ProjTransform(
            self.proj_lonlat, self.proj_dest)

#         process = sp.Popen(shlex.split(f'./telemetry/telemetry {self.telemetry_bin} -'), stdout=sp.PIPE)
#         np.set_printoptions(precision=8, suppress=True, linewidth=151)

# #        stdout, stderr = process.communicate()
# #        print (stdout)
# #        return
# #        
#         start = time.monotonic()
#         all_gps5 = np.zeros((math.ceil(count / fps * 20), 12))
#         all_gps_timestamp = np.zeros((math.ceil(count / fps)), dtype='datetime64[us]')
#         position = 0

#         for tps, line in enumerate(process.stdout):
#             accl, gyro, gps5, gps_accuracy, gps_fix, gps_timestamp, temperature, faces = line.decode().rstrip('\n').split('|')

#             date = datetime.strptime(gps_timestamp, '%y%m%d%H%M%S.%f')
#             all_gps_timestamp[tps] = date

#             matrix = np.matrix(gps5)
#             accl = np.matrix(accl)

#             nb_step = matrix.shape[0]

#             for i, mean_accl in enumerate(np.array_split(accl, nb_step)):
#                 all_gps5[position + i, 7:10] = np.mean(mean_accl, axis=0)

#             all_gps5[position:position+nb_step, 0] = np.linspace(tps * self.fps_rounded, (tps + 1) * self.fps_rounded, nb_step, endpoint=False)
#             all_gps5[position:position+nb_step, 1:6] = matrix
#             all_gps5[position:position+nb_step, 6] = np.arange(position, position+nb_step)
#             position += nb_step

        process = sp.Popen(shlex.split(f'./telemetry/telemetry {self.telemetry_bin} -'), stdout=sp.PIPE)
        np.set_printoptions(precision=8, suppress=True, linewidth=151)

#        stdout, stderr = process.communicate()
#        print (stdout)
#        return
#        
        start = time.monotonic()
        # lat, lon, speed, slope, hor, angle, time, distance
        self.data = np.zeros((count, 8))

        position = 0

        all_gps5 = np.zeros((count, 6))
        all_accl = np.zeros((count, 3))
        while position < count:
            accl = read_inertia(process.stdout, False)
            if accl is None:
                break
            gyro = read_inertia(process.stdout)
            gps5 = read_gps5(process.stdout)
            gps_accuracy = read_int(process.stdout)
            gps_fix = read_int(process.stdout)
            gps_timestamp = read_timestamp(process.stdout)
            temperature = read_float32(process.stdout)
            faces = read_faces(process.stdout)

            date = datetime.strptime(gps_timestamp, '%y%m%d%H%M%S.%f')
            date = date.hour * 3600 + date.minute * 60 + date.second + date.microsecond / 1000000.
            all_gps5[position:position + self.fps_rounded, 5] = np.linspace(date, date + 1, self.fps_rounded, endpoint=False)

            offset = 1. / (self.fps_rounded * 2)
            indices = np.linspace(offset, 1 - offset, self.fps_rounded)

            nb_step = gps5.shape[0]
            offset = 1. / (nb_step * 2)
            indices_gps5 = np.linspace(offset, 1 - offset, nb_step)
            spline = interp1d(indices_gps5, gps5, axis=0, bounds_error=False, fill_value="extrapolate")
            all_gps5[position:position + self.fps_rounded,:5] = spline(indices)

            for i, mean_accl in enumerate(np.array_split(accl, self.fps_rounded)):
                all_accl[position+i] = np.mean(mean_accl, axis=0)

            position += self.fps_rounded

        self.data[:,0:2] = all_gps5[:,0:2]
        self.data[:,2] = all_gps5[:,4]
        self.data[:,6] = all_gps5[:,5]

        print('csv', time.monotonic() - start)
        start = time.monotonic()

        thres_move = 1
        previous_index = 0
        next_index = 0
        self.zone = np.zeros((count, 3))
        self.zone[:,0] = np.arange(count)
        self.zone[:,1] = np.arange(count)
        for i in range(count):
            position = self.data[i,0:2]

            for next_index in range(next_index, count):
                if fast_haversine(position, self.data[next_index, 0:2], 'm') > thres_move:
                    break
                self.zone[next_index][0] = i
            
            self.zone[i][1] = next_index

        self.zone[:,2] = self.zone[:,1] - self.zone[:,0]
        print(np.argmax(self.zone[:,2]))
        print(self.zone[np.argmax(self.zone[:,2])])
        print('segmentation', time.monotonic() - start)
        start = time.monotonic()
        sys.exit(0)


        #     next_position = self.data[next_index]
        #     first = self.transform_lonlat_osm.forward(mapnik.Coord(position[1], position[0]))
        #     second = self.transform_lonlat_osm.forward(mapnik.Coord(next_position[1], next_position[0]))
        #     self.data[i][5] = math.atan2(second.y - second.x, first.y - first.x)



        is_running = self.data[:,2] > (1 / 3.6)
        is_running[0] = True
        is_running[-1] = True
        is_running_diff = np.concatenate((np.zeros((1)), np.diff(is_running.astype(int))))
        starting = np.argwhere(is_running_diff == 1)
        stopping = np.argwhere(is_running_diff == -1)

        running_length = stopping[1:] - starting[:-1]
        true_runs = np.concatenate((np.array([[True]]), running_length > 3600, np.array([[True]])))
        stopping = stopping[true_runs[:-1]]
        starting = starting[true_runs[1:]]

        print(stopping, starting, starting - stopping)
        self.pause = np.vstack((stopping, starting, starting - stopping))

        print(self.pause[:10])
        true_pause = self.pause[np.argwhere(self.pause[:,2] > 3600)]
        print(true_pause[:10])
        print(np.sum(self.pause[:,2]))
        return

        print('csv', time.monotonic() - start)
        start = time.monotonic()

        fast_accl = np.zeros((all_accl.shape))
        fast_accl[0] = all_accl[0]
        coeff = 0.99
        for i, a in enumerate(all_accl):
            if i > 0:
                fast_accl[i] = fast_accl[i-1] * coeff + a * (1 - coeff)
        print(fast_accl[10000:10010], all_accl[10000:10010])

        fast_accl = all_accl.copy()
        coeff = 0.99
        for i in range(120):
            fast_accl[1:] = fast_accl[:-1] * coeff + all_accl[1:] * (1 - coeff)
        print(fast_accl[10000:10010], all_accl[10000:10010])

        fast_accl = nd.gaussian_filter1d(all_accl, self.fps_rounded, axis=0)
        print('csv fast filter', time.monotonic() - start)
        start = time.monotonic()
        slow_accl = nd.uniform_filter1d(all_accl, self.fps_rounded * 60, axis=0)
        print('csv slow filter', time.monotonic() - start)
        start = time.monotonic()
        print(slow_accl[:10])
        self.data[:,4] = slow_accl[:,1] / slow_accl[:,0]
        self.data[:,3] = -fast_accl[:,2] / fast_accl[:,0]
        min_slope = np.argmin(self.data[:,3])
        print(np.argmin(self.data[:,3]))
        print(np.argmax(self.data[:,3]))
        print("data", self.data[min_slope - 2:min_slope+3])
        print("slow", slow_accl[min_slope - 2:min_slope+3])
        print("fast", fast_accl[min_slope - 2:min_slope+3])
        print("nothing", all_accl[min_slope - 2:min_slope+3])

        # thres_move = 1
        # next_index = 0
        # for i in range(count):
        #     position = self.data[i,0:2]
        #     while next_index < count - 1 and haversine(position, self.data[next_index, 0:2], unit='m') < thres_move:
        #         next_index += 1
            
        #     next_position = self.data[next_index]
        #     first = self.transform_lonlat_osm.forward(mapnik.Coord(position[1], position[0]))
        #     second = self.transform_lonlat_osm.forward(mapnik.Coord(next_position[1], next_position[0]))
        #     self.data[i][5] = math.atan2(second.y - second.x, first.y - first.x)
        print('end init', time.monotonic() - start)
        start = time.monotonic()

    def __iter__(self):
        self.elapsed = 0
        return self


    def __next__(self):
        elapsed = self.elapsed
        self.elapsed += 1

        lat, lon, speed, slope, hor, angle, time, distance = self.data[elapsed]

        return elapsed, distance, lat, lon, slope, angle, speed, 0

# class Decode(object):
#     def __init__(self, telemetry_bin, srs, fps):
#         self.fps = round(fps)
#         EPSG_4326 = '+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs'
#         self.proj_lonlat = mapnik.Projection(EPSG_4326)
#         self.proj_dest = mapnik.Projection(srs)
#         self.transform_lonlat_osm = mapnik.ProjTransform(
#             self.proj_lonlat, self.proj_dest)
#         self.telemetry_bin = telemetry_bin
#         self.thres_slope = 5
#         self.factor_gravity = 0.99 ** (1 / self.fps)
#         self.factor_direction = 0.99 ** (1 / self.fps)
#         self.factor_vibration = 0.9 ** (1 / self.fps)
#         self.integration_distance = 100
#         self.sto_merc = lambda x : to_mercator(self.transform_lonlat_osm, x)
#         self.vto_merc = np.vectorize(self.sto_merc)

#     def __iter__(self):
#         self.last_gps5 = np.zeros((1,5))
#         self.elapsed = 0
#         self.last_accl = np.zeros((1,3))
#         self.gps_timestamp = ""
#         self.cum_distance = 0
#         self.current_vector = np.zeros(3)
#         self.process = sp.Popen(shlex.split(f'./telemetry/telemetry {self.telemetry_bin} -'), stdout=sp.PIPE)
#         self.gps5 = np.matrix("0 0 0 0 0")
#         self.accl = np.matrix("0 0 0")
#         self.current_gravity = np.zeros(3)
#         self.accl = np.matrix("0 0 0")
#         self.current_vibration = np.zeros(3)
#         self.current_vibration_gyro = np.zeros(3)
#         self.gps_per_frame = 0
#         self.accl_per_frame = 0
#         return self

#     def __next__(self):
#         elapsed = self.elapsed
#         self.elapsed += 1
#         if elapsed % self.fps == 0:
#             line = self.process.stdout.readline().decode()
#             if not line:
#                 raise StopIteration
#             if line[-1] == '\n':
#                 line = line[:-1]

#             accl, gyro, gps5, gps_accuracy, gps_fix, gps_timestamp, temperature, faces = line.split('|')
#             self.gps5 = np.matrix(gps5)
#             self.accl = np.matrix(accl)
#             self.gyro = np.matrix(gyro)
#             if elapsed == 0:
#                 self.current_gravity = self.accl[0].getA1()
#                 self.last_gps5 = self.gps5[0]

#             self.gps_per_frame = self.gps5.shape[0] / self.fps
#             self.accl_per_frame = self.accl.shape[0] / self.fps
#             self.gps_timestamp = gps_timestamp

#         gps_timestamp = self.gps_timestamp
#         intraframe = elapsed % self.fps
#         start_gps = int(intraframe * self.gps_per_frame)
#         end_gps = int((intraframe + 1) * self.gps_per_frame)
#         gps5 = np.concatenate((self.last_gps5, self.gps5[start_gps:end_gps,:]))
#         avg_gps5 = np.mean(gps5, axis = 0).getA1()

#         delta = self.sto_merc(gps5[-1].getA1()) - self.sto_merc(gps5[0].getA1())
#         distance = np.linalg.norm(delta[0:2])
#         self.cum_distance += distance
#         if distance > 1. / self.fps:
#             direction = delta / distance
#             self.current_vector = self.factor_direction * self.current_vector + direction * (1 - self.factor_direction)
#         slope = math.atan2(self.current_vector[2], distance)
#         angle = math.atan2(self.current_vector[1], self.current_vector[0]) 

#         start_accl = int(intraframe * self.accl_per_frame)
#         end_accl = int((intraframe + 1) * self.accl_per_frame)
#         accl = np.concatenate((self.last_accl, self.accl[start_accl:end_accl,:]))
#         gyro = self.gyro[start_accl:end_accl,:]
        
#         self.current_gravity = self.factor_gravity * self.current_gravity + (1 - self.factor_gravity) * np.mean(accl[1:], axis=0).getA1()
#         self.current_vibration = self.factor_vibration * self.current_vibration + (1 - self.factor_vibration) * np.mean(np.diff(accl[1:], axis=0), axis=0).getA1()
#         self.current_vibration_gyro = self.factor_vibration * self.current_vibration_gyro + (1 - self.factor_vibration) * np.mean(gyro, axis=0)

#         horizon = math.atan2(self.current_gravity[1], self.current_gravity[0]) * 180 / math.pi
#         tanguage = math.atan2(self.current_gravity[2], self.current_gravity[0]) * 180 / math.pi

#         self.last_gps5 = gps5[-1]
#         self.last_accl = accl[-1]
#         return elapsed, self.cum_distance, avg_gps5, gps_timestamp, angle, slope, horizon, tanguage, self.current_vector

