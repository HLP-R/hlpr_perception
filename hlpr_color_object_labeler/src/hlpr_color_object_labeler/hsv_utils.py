from colorsys import rgb_to_hsv, hsv_to_rgb

def in_range(x, x_range):
    min_x, max_x = x_range
    return x > min_x and x < max_x

def in_hue_range(h, h_range):
    min_h, max_h = h_range
    if min_h < max_h:
        return in_range(h, h_range)
    else:
        return h > min_h or h < max_h

def in_hsv_range(h, s, v, h_range=None, s_range=None, v_range=None):
    in_hue = in_hue_range(h, h_range) if h_range != None else True
    in_sat = in_range(s, s_range) if s_range != None else True
    in_val = in_range(v, v_range) if v_range != None else True

    return in_hue and in_sat and in_val

def in_rgb_range(r, g, b, r_range=None, g_range=None, b_range=None):
    in_r = in_range(r, r_range) if r_range != None else True
    in_g = in_range(g, g_range) if g_range != None else True
    in_b = in_range(b, b_range) if b_range != None else True

    return in_r and in_g and in_b
