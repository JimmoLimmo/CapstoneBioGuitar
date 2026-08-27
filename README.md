# Bio Inspired Guitarist

A machine learning pipeline that predicts guitar hand landmark positions from finger placement inputs.

---

## Overview

**Bio Inspired Guitarist** is a capstone project that applies machine learning to model and predict hand biomechanics during guitar playing. Given a set of inputs describing a guitarist's finger placement — string number, fret number, and finger index — the system predicts the expected 3D positions of hand landmarks using a trained neural network.

The project uses **MediaPipe** for hand landmark data collection, **scikit-learn's MLPRegressor** for training a multi-output regression model.
---

## Setup

### Clone the Repository

```bash
git clone https://github.com/wprobst1/CapstoneBioGuitar
```

### Dependencies

Install required Python packages:

```bash
pip install -r requirements.txt
```

---

## Usage

### 1. Prepare Data

Place your input/output CSV files in Learning/train.csv:

- Inputs — columns: `string`, `fret`, `finger`
- Outputs — 18 columns representing x/y/z coordinates for 5 fingertips + wrist

### 2. Train the Model

```bash
python3 main.py
```
