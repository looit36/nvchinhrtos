import json
import math

def analyze(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    # Extract EstXY data
    xy_data = data['telemetries']['EstXY']['data']
    x_vals = xy_data[0]
    y_vals = xy_data[1]
    t_vals = xy_data[2]
    
    num_samples = len(x_vals)
    start_time = t_vals[0]
    end_time = t_vals[-1]
    total_time = end_time - start_time
    
    distance = 0.0
    for i in range(1, num_samples):
        dx = x_vals[i] - x_vals[i-1]
        dy = y_vals[i] - y_vals[i-1]
        distance += math.sqrt(dx**2 + dy**2)
        
    final_x = x_vals[-1]
    final_y = y_vals[-1]
    
    return {
        'num_samples': num_samples,
        'total_time_sec': total_time,
        'avg_sample_rate': num_samples / total_time if total_time > 0 else 0,
        'distance_mm': distance, # assuming unit is mm
        'final_x': final_x,
        'final_y': final_y
    }

hse_stats = analyze('hse.json')
hsi_stats = analyze('hsi.json')

print("=== SO SÁNH KẾT QUẢ RUN ===")
print(f"{'Metric':<20} | {'HSE (Thạch anh ngoài)':<25} | {'HSI (Thạch anh nội)':<25}")
print("-" * 75)
for key in hse_stats.keys():
    print(f"{key:<20} | {hse_stats[key]:<25.4f} | {hsi_stats[key]:<25.4f}")
