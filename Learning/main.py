from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import OneHotEncoder, StandardScaler
from sklearn.model_selection import train_test_split
import pandas as pd
import numpy as np
import joblib

df = pd.read_csv("train.csv").dropna()

df["string"] = pd.to_numeric(df["string"], errors="coerce")
df["finger"] = pd.to_numeric(df["finger"], errors="coerce")
df["fret"]   = pd.to_numeric(df["fret"], errors="coerce")

df = df.dropna()

df["string"] = df["string"].astype(int)
df["finger"] = df["finger"].astype(int)


ohe_string = OneHotEncoder(sparse_output=False, handle_unknown="ignore")
ohe_finger = OneHotEncoder(sparse_output=False, handle_unknown="ignore")

string_encoded = ohe_string.fit_transform(df[["string"]])
finger_encoded = ohe_finger.fit_transform(df[["finger"]])

fret = df[["fret"]].values
X = np.hstack([string_encoded, fret, finger_encoded])

output_cols = [
    "Wx", "Wy", "Wz",
    "Tx", "Ty", "Tz",
    "Ix", "Iy", "Iz",
    "Mx", "My", "Mz",
    "Rx", "Ry", "Rz",
    "Px", "Py", "Pz",
]

y = df[output_cols].values

scaler_X = StandardScaler()
scaler_y = StandardScaler()

X = scaler_X.fit_transform(X)
y = scaler_y.fit_transform(y)

# Test / Train split
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42
)

# Number of Epochs
model = MLPRegressor(
    hidden_layer_sizes=(128, 64),
    max_iter=500,
    random_state=42
)

model.fit(X_train, y_train)

score = model.score(X_test, y_test)
print(f"\nR² score: {score:.4f}")

def predict_landmarks(string, fret, finger):
    string = int(string)
    finger = int(finger)
    fret   = float(fret)

    string_enc = ohe_string.transform([[string]])
    finger_enc = ohe_finger.transform([[finger]])
    fret_arr   = np.array([[fret]])

    X_input = np.hstack([string_enc, fret_arr, finger_enc])
    X_input = scaler_X.transform(X_input)

    predicted_scaled = model.predict(X_input)
    predicted = scaler_y.inverse_transform(predicted_scaled).reshape(6, 3)

    return predicted

predict_landmarks(string=3, fret=5, finger=2)

joblib.dump(model, "guitar_model.pkl")
joblib.dump(ohe_string, "ohe_string.pkl")
joblib.dump(ohe_finger, "ohe_finger.pkl")
joblib.dump(scaler_X, "scaler_X.pkl")
joblib.dump(scaler_y, "scaler_y.pkl")

np.savez(
    "guitar_weights.npz",
    *model.coefs_,
    *model.intercepts_,
    n_layers=len(model.coefs_)
)

np.save("string_categories.npy", ohe_string.categories_[0])
np.save("finger_categories.npy", ohe_finger.categories_[0])