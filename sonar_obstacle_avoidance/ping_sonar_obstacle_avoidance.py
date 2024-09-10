

"""
Sources:
    * Repo: https://github.com/bluerobotics/pingpython/blob/master/examples/simplePingExample.py
    * api: https://docs.bluerobotics.com/pingpython/classbrping_1_1ping1d_1_1Ping1D.html
1. Connect to ping
1.1 Set devices mode to auto, change gain setting, range, ping rate
1.2 (optional) set speed of sound for air: 343 m/s and 1481 m/s in water
2. Get profile, distance, range, confidence
3. Convert distances from mm to m
4. Get time delta (dt) to calculate object velocity
5. (optional) add a filter to reduce noise. NOTE: DON'T BOTHER FOR NOW AS THE CONFIDENCE AND DISTANCES DO NOT NECESSARILY REFER TO ONE OBJECT ONLY
6. Filter/remove confidence below threshold (using a low pass filter). todo: Annette
7. Ignore measurements <= min distance and >= max distance
8. Save to CSV (https://discuss.bluerobotics.com/t/retrieve-ping-sonar-data-for-analysis/11795/2)
9. (optional) Plot results

Notes:
    * if auto mode does not work, switch to manual and adjust gains

In-person tests:
    * point it without facing any obstacles
"""