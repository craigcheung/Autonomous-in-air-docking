import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import time as t

def filter_data(signal, filename):
    #Customizable
    #cutoff_freq = [2, 10] #array for bandstop filter, single value for lowpass

    cutoff_freq = 5
    order = 1

    #filename = '2_26m.txt'
    #col 1 is x position, col 2 is y position, col 3 is z velocity
    #data comes from camera (and we are choosing distances)

    #arr = np.loadtxt(filename)
    
    arr = signal

    data1 = [item[0] for item in arr]  
    data2 = [item[1] for item in arr] 
    data3 = [item[2] for item in arr]

    n = len(arr)
    t = 1/30
    time = np.linspace(0.0, n*t, n, endpoint=False)
    sampling_freq = 1/t

    nyquist_freq = 0.5 * sampling_freq
    normalized_cutoff_freq = cutoff_freq / nyquist_freq
    #normalized_cutoff_freq = [x / nyquist_freq for x in cutoff_freq]

    b, a = butter(order, normalized_cutoff_freq, btype='low')


    filtered_data1 = filtfilt(b, a, data1)
    filtered_data2 = filtfilt(b, a, data2)
    filtered_data3 = filtfilt(b, a, data3)

    #mse1 = np.mean((data1 - filtered_data1)**2)
    #mse2 = np.mean((data2 - filtered_data2)**2)
    #mse3 = np.mean((data3 - filtered_data3)**2)

#    print("Mean Squared Error (Data Col 1):", mse1)
#    print("Mean Squared Error (Data Col 2):", mse2)
#    print("Mean Squared Error (Data Col 3):", mse3)#

#    plt.figure(figsize=(10, 5))

#    plt.plot(time, data1, label='Original Data Col 1')
#    plt.plot(time, filtered_data1, label='Filtered Data Col 1')

#    plt.plot(time, data2, label='Original Data Col 2')
#    plt.plot(time, filtered_data2, label='Filtered Data Col 2')
    
#    plt.plot(time, data3, label='Original Data Col 3')
#    plt.plot(time, filtered_data3, label='Filtered Data Col 3')
    
#    plt.title('Original vs Filtered Data')
#    plt.xlabel('Time')
#    plt.ylabel('Amplitude')
#    plt.legend()
#    plt.grid(True)
#    plt.show()


#    mse_original1 = np.mean((data1 - np.mean(data1))**2)
#    mse_original2 = np.mean((data2 - np.mean(data2))**2)
#    mse_original3 = np.mean((data3 - np.mean(data3))**2)

#    percentage_filtered_out1 = (1 - (mse1 / mse_original1)) * 100
#    percentage_filtered_out2 = (1 - (mse2 / mse_original2)) * 100
#    percentage_filtered_out3 = (1 - (mse3 / mse_original3)) * 100

#    print("Percentage of Noise Filtered Out (Data Col 1):", percentage_filtered_out1, "%")
#    print("Percentage of Noise Filtered Out (Data Col 2):", percentage_filtered_out2, "%")
#    print("Percentage of Noise Filtered Out (Data Col 3):", percentage_filtered_out3, "%")

#    usable_data1 = data1 - filtered_data1
#    usable_data2 = data2 - filtered_data2
#    usable_data3 = data3 - filtered_data3

    new_filename = 'filtered_' + filename

    #COMMENT THIS OUT IF YOU WANT THE FILE OF FILTERED DATA TO BE RETURNED
    
    with open(new_filename, 'w') as file:
        for i in range(len(filtered_data1)):
            file.write(f" {float(filtered_data1[i]):.3f} {float(filtered_data2[i]):.3f} {float(filtered_data3[i]):.3f}\n")
    
            
#    print("Usable data has been written to", new_filename)
#    print(filtered_data1)
    x_pos5 = [item for item in filtered_data1[-1:-6:-1]]
    y_pos5 = [item for item in filtered_data2[-1:-6:-1]]
    total_dist5 = [item for item in filtered_data3[-1:-6:-1]]
    x_pos_avg = sum(x_pos5) / 5
    y_pos_avg = sum(y_pos5) / 5
    total_dist_avg = sum(total_dist5) / 5


    #WHEN THE IMPLEMENTATION IS GOING TO HAPPEN, MAKE SURE TO RETURN THE AVERAGE OF THE LAST 5 POINTS WHICH WILL BE RETURNED TO THE MAVLINK SECTION

    return [x_pos_avg, y_pos_avg, total_dist_avg]
