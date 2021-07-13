import numpy as np

A0 = np.array([[0,1,0],[0,0,1],[-0.1, -0.5, -0.7]])
B0 = np.array([[0,0,1]]).reshape(-1,1)
A1=np.array([[0,1,0,0],[-5,-0.5,3,-1],[0,0,0,1],[0,0,-5,0]])
B1=np.array([0,1,0,0]).reshape(-1,1)
A2=np.array([[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1],[0,-11.41,0,-40.85,1.35,0],[0,176.81,0,403.48,-13.31,0],[0,0,0,0,0,-8.17]])
B2=np.array([[0,0],[0,0],[0,0],[55.53,55.53],[-548.6,-548.6],[-138.92,138.92]])
A3=np.array([[1.1,0.3,-1.5],[0.1,3.5,2.2],[0.4,2.4,-1.1]])
B3=np.array([[0.1,0],[0,1.1],[1.0,1.0]])
A4=np.array([[0,1,0],[0,0,1],[-24,-26,-9]])
B4=np.array([0,0,24])
A5=np.array([[-4.1979  ,  4.8048  , -9.8045],
    [6.7730 ,   0.6202 ,  -0.9885],
   [-5.3608  ,  7.1835 ,  -6.0392]])
B5=np.array([[2.9080 ,  -1.0582  ,  1.0984],
    [0.8252 ,  -0.4686 ,  -0.2779],
    [1.3790  , -0.2725  ,  0.7015]])
A6=np.array([0])
B6=np.array([0])
A7=np.array([[-100.0000 ,   5.6397 ,  -5.9813 , -16.6273 ,  19.2846],
  [ -7.0770 ,   0.6696  ,  0.4578 , -19.5841 ,  10.4012],
  [-16.4717 , -26.6736 ,  -5.2399 , -23.1280  , -0.4006],
  [-31.5411  , 22.5498 , -35.0042 , -10.6711 ,  -0.6954],
  [ 10.1595  ,  7.0036 ,  -5.7130 , -40.0527 ,  100.0000]])
B7=np.array([0,0,5,0,0])
A8=np.reshape(np.array(range(64)),[8,8])
B8=np.array(list(reversed(range(8))))+8
A9=np.array([[12.5013 ,  13.9034  ,  8.2931  ,  5.6405  ,  2.7693  ,  3.7539 ,  -5.7323 ,  -7.2220  , -2.0305  ,  4.4631],
   [-0.7688 ,   4.3802 ,   6.1936  ,  9.5057 ,  10.4407 ,   5.5619 ,  -4.3084 ,  -3.5484  ,  0.6042  , -4.0679],
  [ -7.2199 ,  -8.5055 ,   1.2731 ,  -7.9719 ,   9.8708 ,   4.6761 ,   0.8925 ,   0.2180  ,  2.6965  ,  1.4712],
  [  9.5063 ,  -4.1305 ,   3.6752 ,  13.2346 ,   6.1080 ,  -4.8064 ,  -2.2224 ,   4.8724  , -7.8621  , 14.0772],
  [ -0.2565 ,  -1.1710 ,  -2.5920 ,   9.3928 ,  -0.5348 ,  -2.4688 ,  13.0845 ,  -3.4447  , -3.4379  ,  3.6701],
  [ -3.9577 ,  10.5299 ,   8.6173 ,   2.1698 ,  10.2895 ,   1.7731 ,   0.7552 ,   5.0711  , 10.0254  ,  3.0284],
  [  0.0978 ,  -9.6149 ,  -5.2761 ,   0.8965 ,   3.3206 ,  -4.2378 ,  -5.3796 ,   7.7804  , -9.2695  , -4.2101],
  [ -7.5886 ,  -8.9244 ,   7.1694 ,   1.1696 ,  -1.2318 ,  11.1077 ,  12.6220 ,  -4.4563  , 13.2214  ,  2.2224],
  [ -6.7007 ,  -5.7752 ,  -5.4122 ,  -2.3413 ,  13.4750 ,  -5.1309 ,  14.4937 ,  -7.0646  ,  8.2583  ,  5.6015],
  [ 13.5513 ,   6.2279 ,  -0.7879 ,   2.7127 ,  11.8986 ,  -4.3520 ,   0.9717 ,  -2.5831  ,  2.2152  ,  6.9784]])
B9=np.array([[1,1],[2,4],[3,9],[4,16],[5,25],[6,-36],[7,-49],[8,-64],[9,-81],[10,0]])
A10=np.array([[-37.2809 ,  65.2356  , 92.9789, -231.9280, -212.3655 , 114.5362 , -27.3047 , -90.4726 ,  33.6213 ,  31.9949 , -44.6995 , -109.6593 , -218.6022 ,  58.9433, -152.1027],
[  -23.6455 ,  32.7060 ,  23.9763 ,   7.9934 , -50.4586 , -62.9091 , 157.6300 , -46.7715 , -90.4654 , -55.8294 ,  10.9659 , -49.3010 ,-132.7043 ,  -6.2791 , -72.3631],
[  202.3691 , 108.2634 , -69.0361 , -94.8481 ,-127.0594 ,-120.3850 , -48.0937 , -12.4890 , -28.8256 , -31.1429 , 112.8736 , -18.0739 ,-144.1014 ,-202.1959 , -59.3250],
[ -225.8354 , 100.6077 , -65.1554 ,  41.1491 , -38.2585 , -25.3945 ,  32.7512 , 147.8958 ,  35.0063 , -57.0010 , -28.9963 ,   4.5841 ,  40.1844 , -98.2132 ,  40.1336],
[  222.9446 , -65.0908 , 119.2102 ,  67.6978 ,  64.8679 ,-142.8647 ,  66.4734 , -86.0816 ,-183.5859 ,-102.5734 , 126.1551 ,  -6.3783 , 147.0201 ,  61.2511 ,  94.2133],
[   33.7564 ,  25.7056 ,-161.1830 ,  85.7733 ,  82.5727 ,  -2.0858 ,   8.5189 ,  78.4668 , 103.5976 , -90.8746 ,  47.5425 ,  61.1335 , -32.6814 ,  -5.4886 ,  30.0486],
[  100.0061 , -94.4378 ,  -2.4462 , -69.1159 ,-101.4944 , -56.0665 ,  88.0953 ,  30.8623 , 242.4461 , -20.9897 , 117.4117 ,  10.9318 ,  81.2323 ,-111.8732 , -37.3071],
[ -166.4164 ,-132.1789 ,-194.8847 ,  44.9378 , -47.1070 , 217.7779 ,  32.3213 , -23.3860 ,  95.9401 ,-169.8864 ,  12.6947 , 181.4015 ,  54.5540 , -62.6379 ,  81.5489],
[  -59.0035 ,  92.4826 , 102.0498 ,  10.0633 ,  13.7025 , 113.8465 , -78.4146 ,-105.6973 , -31.5772 ,  60.7601 , -65.6816 ,  31.2024 ,-105.1632 ,  24.9518 ,  79.8887],
[  -27.8064 ,   0.0050 ,  86.1716 ,  82.6070 , -29.1863 ,-249.6887 ,-180.5373 , -28.4141 ,  42.8623 , -11.7798 ,-148.1399 , 180.4494 ,  39.7467 , -99.3019 ,  12.0205],
[  42.2716  , -5.4919  ,  0.1162  , 53.6157  , 30.1819  , 44.1327  ,185.8593  , -8.6690  ,  -103.5985  , 69.9160  , 15.5489  ,-72.3121  ,-75.1895  , 97.4950  , 57.1248],
[ -167.0201 ,  91.1127 ,  -7.0837 ,  89.7888 ,  39.9931 ,-139.8138 , -60.4530 ,-146.9395 , 187.7865 ,  26.9649 ,  81.8551 ,  52.6547 , 151.6267 , -64.0710 ,  41.2796],
[   47.1634 ,  59.4584 ,-248.6284 , -13.1938 , -92.9962 , -25.5055 ,  10.3360 ,  19.2182 ,  94.0704 ,  49.4287 , -29.2588 , -26.0251 ,  -3.2567 , 180.8863 , -98.6962],
[ -121.2847 ,  35.0201 ,  58.1172 , -14.7201 , -17.6830 ,  16.4404 ,  56.3167 , -82.2293 ,  78.7346 ,-148.3121 , -54.0786 ,  60.0143 , 163.6000 ,-107.9866 ,  75.9568],
[    6.6190 , 125.0251 ,-219.2435 , 100.7773 ,-213.2095 ,  74.7734 ,  11.3597 ,  -9.4241 , -87.5874 ,-102.0264 , -30.8642 ,  59.3931 , -42.5058 ,  19.9189 , -65.7201]])
B10=np.array([[1,0,0],[0,1,0],[0,0,1],[1,0,0],[0,1,0],[0,0,1],[1,0,0],[0,1,0],[0,0,1],[1,0,0],[0,1,0],[0,0,1],[1,0,0],[0,1,0],[0,0,1]])
A11 = np.array([[-0.4125, -0.0248, 0.0741, 0.0089, 0, 0], \
                  [101.5873, -7.2615, 2.7608, 2.8068, 0, 0],
                  [0.0704, 0.0085, -0.0741, -0.0089, 0, 0.02],
                  [0.0878, 0.2672, 0, -0.3674, 0.0044, 0.3962],
                  [-1.8414, 0.099, 0, 0, -0.0343, -0.033],
                  [0, 0, 0, -359.0, 187.5364, -87.0316]])
B11 = np.array([[-0.0042, 0.0064], [-1.0360, 1.5849], [0.0042, 0], [0.1261, 0], [0, -0.0168], [0, 0]])

all_test_case = [[A0,B0], [A1,B1], [A2,B2], [A3,B3], [A4,B4], [A5,B5], [A6,B6], [A7,B7], [A8,B8], [A9,B9], [A10,B10], [A11,B11]]