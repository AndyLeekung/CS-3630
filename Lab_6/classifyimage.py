import numpy as np
from sklearn.externals import joblib
from skimage import io, feature, filters, exposure, color

clf = joblib.load('classifier.pkl')

def classify_image(raw_img):
    features = extract_image_features([np.array(raw_img)])
    return clf.predict(features)[0]

def extract_image_features(data):
    l = []
    for im in data:
        im_gray = color.rgb2gray(im)
    
        im_gray = filters.gaussian(im_gray, sigma=0.4)
        
        f = feature.hog(im_gray, orientations=10, pixels_per_cell=(48, 48), cells_per_block=(4, 4), feature_vector=True, block_norm='L2-Hys')
        l.append(f)
    
    feature_data = np.array(l)
    return(feature_data)