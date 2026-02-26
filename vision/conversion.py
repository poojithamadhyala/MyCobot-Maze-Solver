import sympy as sp


def line_gen(ref1, ref2):
    x1, y1 = ref1
    x2, y2 = ref2
    slope = (y2 - y1) / (x2 - x1)
    intercept = y1 - slope * x1
    return slope, intercept


def conversion_func(x_px, y_px):
    # Reference points in the form [pixel, real-world coordinate]
    refx1 = [630, -0.364402]
    refy1 = [820, -0.304190]
    refx2 = [1168, -0.25244]
    refy2 = [222, -0.202547]
    refx3 = [1195, -0.254847]
    refy3 = [781, -0.292475]
    refx4 = [601, -0.352240]
    refy4 = [246, -0.193794]
    # refx1 = [140, -0.3793]
    # refy1 = [286, -0.2876]
    # refx2 = [511, -0.2298]
    # refy2 = [158, -0.2300]
    # refx3 = [270, -0.3337]
    # refy3 = [44, -0.1787]
    # refx4 = [382, -0.2734]
    # refy4 = [413, -0.3507]
    # refx1 = [730,-0.351313 ]
    # refy1= [284,-0.213053]
    # refx2=[1347,-0.240537]
    # refy2=[785,-0.321337]
    # refx3=[780,-0.230559]
    # refy3=[838,-0.210952]
    # refx4=[1289,-0.347356]
    # refy4=[229,-0.333839]
    # Generate slopes and intercepts
    refx1 = [1280,-0.251535]
    refy1 = [231,-0.213836]
    refx2 = [721, -0.367901]
    refy2 = [797, -0.321636]
    refx3 = [1281,-0.253495]
    refy3 = [800,-0.326775]
    refx4 = [728,-0.354706]
    refy4 = [227,-0.212343]
    mx1, cx1 = line_gen(refx1, refx2)
    mx2, cx2 = line_gen(refx3, refx4)

    my1, cy1 = line_gen(refy1, refy2)
    my2, cy2 = line_gen(refy3, refy4)

    # Average slopes and intercepts
    mx = (mx1 + mx2) / 2
    cx = (cx1 + cx2) / 2
    my = (my1 + my2) / 2
    cy = (cy1 + cy2) / 2
    print(f"mx: {mx}, cx: {cx}, my: {my}, cy: {cy}")
    # Calculate real-world coordinates
    x_m = sp.N(mx * x_px + cx)
    y_m = sp.N(my * y_px + cy)

    return x_m, y_m


# Example usage
x_px = 1280 # Example pixel x-coordinate
y_px = 800 # Example pixel y-coordinate
x_m, y_m = conversion_func(x_px, y_px)
print(f"x_m: {x_m}, y_m: {y_m}")

# def mapper(x_px, y_px):
#     # Reference points in the form [pixel, real-world coordinate]
#     refx1 = [140, -0.3793]
#     refy1 = [286, -0.2876]
#     refx2 = [511, -0.2298]
#     refy2 = [158, -0.2300]
#     refx3 = [270, -0.3337]
#     refy3 = [44, -0.1787]
#     refx4 = [382, -0.2734]
#     refy4 = [413, -0.3507]
#
#     # Generate slopes and intercepts
#     mx1, cx1 = line_gen(refx1, refx2)
#     mx2, cx2 = line_gen(refx3, refx4)
#
#     my1, cy1 = line_gen(refy1, refy2)
#     my2, cy2 = line_gen(refy3, refy4)
#
#     # Average slopes and intercepts
#     mx = (mx1 + mx2) / 2
#     cx = (cx1 + cx2) / 2
#     my = (my1 + my2) / 2
#     cy = (cy1 + cy2) / 2
#     print(f"mx: {mx}, cx: {cx}, my: {my}, cy: {cy}")
#     # Calculate real-world coordinates
#     x_m = sp.N(mx * x_px + cx)
#     y_m = sp.N(my * y_px + cy)
#
#     return x_m, y_m
#

# # Example usage
# x_px = 200  # Example pixel x-coordinate
# y_px = 300  # Example pixel y-coordinate
# x_m, y_m = mapper(x_px, y_px)
# print(f"x_m: {x_m}, y_m: {y_m}")
