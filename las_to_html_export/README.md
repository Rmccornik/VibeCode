# LAS to HTML Export Addon for Blender

A professional Blender addon designed to export LiDAR point cloud data (`.las` / `.laz`) into interactive, web-ready HTML reports. This tool leverages Blender's powerful data handling capabilities to visualize and export geospatial point data with advanced chunking, compression, and styling options.

**Target Blender Version:** 5.1 - 5.2+  
**License:** GPL-3.0-or-later

---

## 🚀 Features

*   **Interactive HTML Output:** Generates self-contained HTML files with embedded JavaScript viewers (compatible with Potree/Three.js based viewers).
*   **Large Dataset Support:** Built-in chunking system to handle massive LAS/LAZ files without running out of memory.
*   **Compression Options:** Supports optional compression for generated data blocks.
*   **Customizable Styling:** Configure point size, color schemes (by elevation, intensity, or classification), and background colors directly from the UI.
*   **Blender 5.x Optimized:** Refactored codebase utilizing modern Python type hints and Blender 5.1/5.2 API standards.
*   **Modular Architecture:** Clean separation of concerns (UI, Logic, Preferences, Templates) for easy maintenance and extension.

---

## 📋 Requirements

*   **Blender:** Version 5.1 or 5.2 (or newer compatible versions).
*   **Python Dependencies:**
    *   `laspy`: Required for reading `.las` and `.laz` files.
    *   `numpy`: Required for efficient point cloud data processing.

> **Note:** Ensure `laspy` is installed in the Python environment bundled with your Blender installation.
> ```bash
> # Example for Linux/Mac (adjust path to your blender python)
> /path/to/blender/3.x/python/bin/pip install laspy numpy
> ```

---

## 📦 Installation

1.  **Download:** Clone this repository or download the source code as a ZIP file.
    ```bash
    git clone <repository-url>
    ```
2.  **Prepare:** Ensure the folder structure is intact. The root folder containing `__init__.py` should be named `las_to_html_export`.
3.  **Install in Blender:**
    *   Open Blender.
    *   Go to **Edit** > **Preferences** > **Add-ons**.
    *   Click the **Install...** button.
    *   Navigate to the `las_to_html_export` folder (select the folder itself, not a specific file inside it) and click **Install Add-on**.
    *   Enable the addon by checking the checkbox next to "Import-Export: LAS to HTML Export".

---

## ⚙️ Usage

### 1. Accessing the Tool
Once enabled, the addon can be found in the **Sidebar** of the 3D Viewport:
*   Press `N` to toggle the sidebar.
*   Look for the tab labeled **LAS Export**.

### 2. Configuring Preferences (Optional)
Global settings can be adjusted in **Edit** > **Preferences** > **Add-ons** > **LAS to HTML Export**:
*   **Default Chunk Size:** Set the default number of points per chunk for large files.
*   **Default Output Path:** Configure a standard directory for exports.

### 3. Exporting Data
1.  In the **LAS Export** panel, locate the input file path selector.
2.  Choose your `.las` or `.laz` file.
3.  Adjust export settings:
    *   **Chunking:** Enable/disable splitting large files.
    *   **Color Source:** Choose between Elevation, Intensity, Classification, or RGB.
    *   **Point Size:** Define the render size in the HTML viewer.
4.  Click the **Export to HTML** button.
5.  Wait for the progress bar to complete. The HTML file will be saved to the specified output location.

---

## 🏗️ Project Structure

This addon follows a modular architecture to ensure scalability and maintainability:

```text
las_to_html_export/
├── __init__.py          # Main entry point, bl_info, registration
├── preferences.py       # User preferences (AddonPreferences)
├── properties.py        # Scene properties and data pointers
├── operators.py         # The main export operator logic
├── ui.py                # UI Panel definitions
├── utils/               # Core logic modules
│   ├── __init__.py
│   ├── chunking.py      # Logic for splitting point clouds
│   ├── compression.py   # Data compression utilities
│   ├── point_data.py    # LAS/LAZ reading and processing
│   └── html_gen.py      # HTML template rendering
└── templates/           # Static assets
    └── index.html       # Base HTML template for the viewer
```

---

## 🛠️ Development & Contributing

This project is refactored to adhere to modern Python standards:
*   **Type Hinting:** Extensive use of `typing` module for better IDE support and code clarity.
*   **Docstrings:** Google-style docstrings for all public classes and functions.
*   **Separation of Concerns:** UI code is strictly separated from data processing logic.

### Running Tests
*(If test suite is available)*
Run tests via pytest in the project root:
```bash
pytest tests/
```

### Building for Release
To create a distributable ZIP file:
```bash
zip -r las_to_html_export.zip las_to_html_export/ -x "*.git*" "__pycache__*" "*.pyc"
```

---

## 📄 License

This project is licensed under the **GNU General Public License v3.0 (GPL-3.0-or-later)**.
See the [LICENSE](LICENSE) file for details.

---

## 🆘 Troubleshooting

*   **"ModuleNotFoundError: No module named 'laspy'"**: You must install `laspy` into the specific Python version that Blender uses. Refer to the **Requirements** section.
*   **Export hangs on large files**: Try reducing the **Chunk Size** in the export panel or preferences.
*   **HTML looks empty**: Ensure the input LAS file contains valid point data and check the Blender console (`Window` > `Toggle System Console`) for error messages during export.

---

## 📞 Support

For issues, feature requests, or contributions, please open an issue on the GitHub repository.
