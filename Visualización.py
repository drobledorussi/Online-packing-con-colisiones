import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# 1. 2D Visualization:

def visualize_2d(filename, max_dims):
    df = pd.read_csv(filename)
    layers = df['z1'].unique()
    max_x, max_y, _ = max_dims

    for layer in layers:
        subset = df[df['z1'] == layer]
        fig, ax = plt.subplots()
        ax.set_title(f"Layer at z1 = {layer}")

        for index, row in subset.iterrows():
            rect = patches.Rectangle((row['x1'], row['y1']), row['x2']-row['x1'], row['y2']-row['y1'], edgecolor='black', facecolor='gray', label=row['Box ID'])
            ax.add_patch(rect)

        ax.set_xlim(0, max_x)
        ax.set_ylim(0, max_y)
        ax.set_aspect('equal', 'box')
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        plt.show()

def visualize_xz(filename, max_dims):
    df = pd.read_csv(filename)
    slices = df['y1'].unique()
    max_x, _, max_z = max_dims

    for slice_val in slices:
        subset = df[df['y1'] == slice_val]
        fig, ax = plt.subplots()
        ax.set_title(f"Slice at y1 = {slice_val}")

        for index, row in subset.iterrows():
            rect = patches.Rectangle((row['x1'], row['z1']), row['x2']-row['x1'], row['z2']-row['z1'], edgecolor='black', facecolor='red', label=row['Box ID'])  # Cambio aquí
            ax.add_patch(rect)

        ax.set_xlim(0, max_x)
        ax.set_ylim(0, max_z)
        ax.set_aspect('equal', 'box')
        ax.set_xlabel("X")
        ax.set_ylabel("Z")
        plt.show()

def visualize_yz(filename, max_dims):
    df = pd.read_csv(filename)
    slices = df['x1'].unique()
    _, max_y, max_z = max_dims

    for slice_val in slices:
        subset = df[df['x1'] == slice_val]
        fig, ax = plt.subplots()
        ax.set_title(f"Slice at x1 = {slice_val}")

        for index, row in subset.iterrows():
            rect = patches.Rectangle((row['y1'], row['z1']), row['y2']-row['y1'], row['z2']-row['z1'], edgecolor='black', facecolor='blue', label=row['Box ID'])
            ax.add_patch(rect)

        ax.set_xlim(0, max_y)
        ax.set_ylim(0, max_z)
        ax.set_aspect('equal', 'box')
        ax.set_xlabel("Y")
        ax.set_ylabel("Z")
        plt.show()


# 2. 3D Visualization:

# =============================================================================
# def visualize_3d(filename, max_dims):
#     df = pd.read_csv(filename)
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     max_x, max_y, max_z = max_dims
# 
#     for index, row in df.iterrows():
#         ax.bar3d(row['x1'], row['y1'], row['z1'], row['x2']-row['x1'], row['y2']-row['y1'], row['z2']-row['z1'], shade=True)
# 
#     ax.set_xlim(0, max_x)
#     ax.set_ylim(0, max_y)
#     ax.set_zlim(0, max_z)
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')
#     plt.show()
# =============================================================================

def visualize_3d(filename, max_dims, angles=None):
    if angles is None:
        angles = [(30, 0), (30, 45), (30, 90), (30, 135)]  # Default angles (elev, azim)

    df = pd.read_csv(filename)
    max_x, max_y, max_z = max_dims

    n = len(angles)
    cols = 2  # 2 columns of plots
    rows = (n // cols) + (1 if n % cols else 0)  # Calculate the number of rows needed

    fig, axs = plt.subplots(rows, cols, figsize=(10, 10), constrained_layout=True)

    if rows == 1 or cols == 1:
        axs = [axs]

    for idx, angle in enumerate(angles):
        ax = axs[idx // cols][idx % cols] = fig.add_subplot(rows, cols, idx + 1, projection='3d')
        
        for index, row in df.iterrows():
            ax.bar3d(row['x1'], row['y1'], row['z1'], row['x2']-row['x1'], row['y2']-row['y1'], row['z2']-row['z1'], shade=True)

        ax.set_xlim(0, max_x)
        ax.set_ylim(0, max_y)
        ax.set_zlim(0, max_z)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        elev, azim = angle
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(f"View angle: elev={elev}, azim={azim}")

    # Remove any unused subplots
    if n < rows * cols:
        for idx in range(n, rows * cols):
            fig.delaxes(axs[idx // cols][idx % cols])

    plt.show()


# Test the functions
#filename = "PalletLogs/Cajas Pallet Pallet 2 - 2023-10-08 16-59-35.csv"  # Replace with your filename
#visualize_2d(filename)
#visualize_3d(filename)
