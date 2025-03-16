# 拟合线属性，取值如下:
uint8 POLY= 0               # 0x0: 三次曲线
uint8 ARC = 1               # 0x1: 圆弧曲线
uint8 fit_property

# 拟合三次曲线.
# y = f(x) = c3*x^3 + c2*x^2 + c1*x + c0
# 拟合圆弧曲线.
# circular curve:  (y-c1)^2 + (x-c0)^2 = c3 * c3
#                  c0 = x_0
#                  c1 = y_0
#                  c2 = sign(-1 or 1)
#                  c3 = r
float64 c3
float64 c2
float64 c1
float64 c0