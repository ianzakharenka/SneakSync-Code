package com.zakha_dev.sneaksync

import android.Manifest
import android.bluetooth.*
import android.bluetooth.le.BluetoothLeScanner
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.location.LocationManager
import android.os.Build
import android.os.Bundle
import android.provider.Settings
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.github.mikephil.charting.charts.LineChart
import kotlinx.coroutines.*
import java.util.*
import java.util.LinkedList
import java.util.Queue
import android.content.res.ColorStateList
import android.graphics.PorterDuff
import android.widget.ImageView

//import androidx.compose.ui.graphics.Color
import android.graphics.Color
import android.widget.AdapterView
import android.widget.Spinner

import kotlin.text.format
import kotlin.text.toFloat
import kotlin.math.sqrt
import kotlinx.coroutines.*

// --- Graph Imports ---
import com.github.mikephil.charting.components.XAxis
import com.github.mikephil.charting.formatter.ValueFormatter
import com.github.mikephil.charting.data.Entry // For individual data points
import com.github.mikephil.charting.data.LineData // For the overall chart data object
import com.github.mikephil.charting.data.LineDataSet // For a set of data (a line in the chart)
import java.text.SimpleDateFormat
import kotlin.text.trim
import android.widget.ArrayAdapter

import com.robgulley.vector.Madgwick
import com.robgulley.vector.Quaternion
import com.robgulley.vector.Vector

import kotlin.time.ExperimentalTime

private const val GRAVITY_MAGNITUDE = 9.80665
@OptIn(ExperimentalTime::class)

class MainActivity : AppCompatActivity() {


    private val characteristicSubscriptionQueue: Queue<BluetoothGattCharacteristic> = LinkedList()
    private var currentGatt: BluetoothGatt? = null // To hold the gatt instance


    // --- UI Elements ---
    private lateinit var statusTextView: TextView
    private lateinit var velocityTextView: TextView
    private lateinit var stepsTextView: TextView
    private lateinit var distanceTextView: TextView
    private lateinit var pressureTextView: TextView

    private lateinit var madgwickFilter: Madgwick
    private var currentOrientationQuaternion: Quaternion = Quaternion.qId
    private val SENSOR_SAMPLE_RATE_HZ = 20.0
    private val MADGWICK_BETAS: List<Double> = listOf(1.0, 0.5)
    private var latestAccel: Vector? = null
    private var latestGyro: Vector? = null

    private var vcount: Int = 0
    private var vsum: Double = 0.0
    private var steps: Int = 0
    private var distance: Double = 0.0
    private var dTime: Double = 0.0
    private var prevdTime: Double = 0.0
    private var prevVelocity: Double = 0.0
    private var yCount: Int = 0
    private var gCount: Int = 0
    private var bCount: Int = 0
    private var ySumPres: Float = 0.0f
    private var gSumPres: Float = 0.0f
    private var bSumPres: Float = 0.0f
    private var maxPres: Float = 1000.0f
    private var prevPres: Float = 0.0f
    private var v: Double = 0.000
    private var maxv: Double = 0.000
    private var prevxyz: Double = 0.000
    private var prevt: Double = System.nanoTime()/1000000000.0
    private var firstV: Boolean = true
    private var connectionTime: Long = 0
    private val accelDataRegex = Regex("""([-\d.]+),([-\d.]+),([-\d.]+)""")
    private val rotDataRegex = Regex("""([-\d.]+),([-\d.]+),([-\d.]+)""")
    private var dotdotdot: Long = 0L
    private var calibrationText = "Calibrating IMU"
    private lateinit var wipeButton: Button
    private lateinit var scanButton: Button
    private lateinit var connectButton: Button
    private lateinit var disconnectButton: Button
    private lateinit var rightToeBackgroundView: ImageView
    private lateinit var rightBallBackgroundView: ImageView
    private lateinit var rightHeelBackgroundView: ImageView
    private val discoveredDevicesList = mutableListOf<BluetoothDevice>()
    private val discoveredDeviceDisplayNames = mutableListOf<String>()

    private lateinit var deviceSpinner: Spinner
    private lateinit var arrayAdapter: ArrayAdapter<String>

    // --- Chart Elements ---
    private lateinit var velocityChart: LineChart
    private var startTimeMillis: Long = 0
    private val TAG = "MainActivityChart"
    private val MAX_VISIBLE_ENTRIES = 1000

    // --- Connection Status for Paragraph ---
    private var isEsp32Connected: Boolean = false
        set(value) {
            field = value
            updateUIBasedOnConnectionStatus()
        }

    // --- BLE Variables ---
    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bluetoothLeScanner: BluetoothLeScanner? = null
    private var esp32Device: BluetoothDevice? = null
    private var bluetoothGatt: BluetoothGatt? = null


    // --- UUIDs ---
    private val SERVICE_UUID = UUID.fromString("4fafc201-1fb5-459e-8fcc-c5c9c331914b")
    private val YPRESSURE_CHAR_UUID = UUID.fromString("f4377745-c73d-4d79-9d55-a50ca9e9f25f")
    private val GPRESSURE_CHAR_UUID = UUID.fromString("beb5483e-36e1-4688-b7f5-ea07361b26a8")
    private val BPRESSURE_CHAR_UUID = UUID.fromString("77c29869-e17c-43af-bbfc-e1e51b126d91")
    private val ACCEL_CHAR_UUID = UUID.fromString("4754dea5-1358-4f9f-88ab-519005cf27b0")
    private val ROT_CHAR_UUID = UUID.fromString("403a2bc1-6ad6-4d2d-9a5d-9871c9d76d45")
    private val CLIENT_CHARACTERISTIC_CONFIG_UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb") // Standard BLE2902 UUID

    fun updateFootColor(imageView: ImageView, red: Int) {
        Log.i(TAG, "updateFootColor called with red: $red")
        var rred: Int = red
        var green = 0
        var blue = 0
        if(red > 500){
            rred = 255
            green = 255
            blue = 255
        } else if (red > 255){
            rred = 255
            green = red-255
            blue = red-255
        }
        if(rred < 0){
            rred = 0
        }
        val targetColor = Color.argb(255, rred, green, blue)
        imageView.imageTintList = ColorStateList.valueOf(targetColor)
    }

    fun getLinearAcceleration(rawAccel: Vector, orientation: Quaternion): Vector {
        val qw = orientation.w
        val qx = orientation.x
        val qy = orientation.y
        val qz = orientation.z

        val gx = 2 * (qx * qz - qw * qy)
        val gy = 2 * (qy * qz + qw * qx)
        val gz = qw * qw - qx * qx - qy * qy + qz * qz

        val estimatedGravityInDeviceFrame = Vector(
            gx * GRAVITY_MAGNITUDE,
            gy * GRAVITY_MAGNITUDE,
            gz * GRAVITY_MAGNITUDE
        )

        val linearAcceleration = Vector(
            rawAccel.x - estimatedGravityInDeviceFrame.x,
            rawAccel.y - estimatedGravityInDeviceFrame.y,
            rawAccel.z - estimatedGravityInDeviceFrame.z
        )
        Log.i(TAG, "Linear Acceleration: X=${linearAcceleration.x}, Y=${linearAcceleration.y}, Z=${linearAcceleration.z}")
        return linearAcceleration
    }


    // --- Permissions Launcher ---
    private val requestPermissionsLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val allGranted = permissions.entries.all { it.value }
        if (allGranted) {
            Toast.makeText(this, "All Bluetooth permissions granted!", Toast.LENGTH_SHORT).show()
            checkLocationService()
        } else {
            Toast.makeText(this, "Bluetooth permissions denied. Cannot scan/connect.", Toast.LENGTH_LONG).show()
            updateStatus("Permissions denied.")
        }
    }

    private val PROMPT_MESSAGE = "Device List                                                                      ▼" // Or use " V" or an actual down arrow char ▼


    // --- Lifecycle Methods ---
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)


        // Initialize UI elements
        statusTextView = findViewById(R.id.statusTextView)
        velocityTextView = findViewById(R.id.velocityTextView)
        distanceTextView = findViewById(R.id.distanceTextView)
        stepsTextView = findViewById(R.id.stepsTextView)
        pressureTextView = findViewById(R.id.pressureTextView)
        rightToeBackgroundView = findViewById(R.id.rightToeBackgroundView)
        rightBallBackgroundView = findViewById(R.id.rightBallBackgroundView)
        rightHeelBackgroundView = findViewById(R.id.rightHeelBackgroundView)


        wipeButton = findViewById(R.id.wipeButton)
        scanButton = findViewById(R.id.scanButton)
        connectButton = findViewById(R.id.connectButton)
        disconnectButton = findViewById(R.id.disconnectButton)
        velocityChart = findViewById(R.id.velocityChart)
        startTimeMillis = System.currentTimeMillis()
        setupVelocityChart()
        Log.d(TAG, "Chart setup complete. Waiting for data...")

        // Initialize Spinner
        deviceSpinner = findViewById(R.id.deviceSpinner)
        discoveredDeviceDisplayNames.add(PROMPT_MESSAGE)

        arrayAdapter = ArrayAdapter(this, R.layout.simple_spinner_item_white_text, discoveredDeviceDisplayNames)
        arrayAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)

        deviceSpinner.adapter = arrayAdapter
        deviceSpinner.setSelection(0,false)

        madgwickFilter = Madgwick(sampleFreq = null, betas = MADGWICK_BETAS)
        Log.d("Madgwick", "Madgwick filter initialized.")
        deviceSpinner.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(parent: AdapterView<*>, view: View?, position: Int, id: Long) {
                if (position == 0) {
                    // User selected the prompt item (e.g., "Select a device...", "Device List V")
                    Log.d(TAG, "Prompt selected in Spinner.")
                    connectButton.isEnabled = false
                    esp32Device = null
                } else {
                    // User selected an actual device (position > 0)
                    val deviceIndexInActualList = position - 1

                    // Safety check: ensure the adjusted index is valid for discoveredDevicesList
                    if (deviceIndexInActualList >= 0 && deviceIndexInActualList < discoveredDevicesList.size) {
                        esp32Device = discoveredDevicesList[deviceIndexInActualList]
                        connectButton.isEnabled = true
                        updateStatus("Selected: ${esp32Device?.name ?: esp32Device?.address}")
                        Log.i(TAG, "Device selected: ${esp32Device?.name ?: esp32Device?.address} (Spinner pos: $position, List index: $deviceIndexInActualList)")
                    } else {
                        Log.e(TAG, "Spinner Error: Adjusted index $deviceIndexInActualList is out of bounds for discoveredDevicesList (size ${discoveredDevicesList.size}). Original spinner position: $position")
                        connectButton.isEnabled = false
                        esp32Device = null
                    }
                }
            }

            override fun onNothingSelected(parent: AdapterView<*>) {
                connectButton.isEnabled = false
                esp32Device = null
            }
        }




        updateUIBasedOnConnectionStatus()

        // Set up button click listeners
        scanButton.setOnClickListener { startBleScan() }
        connectButton.setOnClickListener { connectToDevice() }
        disconnectButton.setOnClickListener { disconnectFromDevice() }
        wipeButton.setOnClickListener { resetAllData() }


        // Initialize Bluetooth adapter
        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothAdapter = bluetoothManager.adapter
        bluetoothLeScanner = bluetoothAdapter?.bluetoothLeScanner


        // Request permissions (this will be done once on app start)
        requestBluetoothPermissions()

    }

    private fun resetGraphData() {
        if (::velocityChart.isInitialized) {
            val chartData = velocityChart.data
            if (chartData != null) {
                chartData.clearValues()
                velocityChart.notifyDataSetChanged()
                velocityChart.invalidate()
                setupVelocityChart()
                Log.i(TAG, "Graph data cleared.")
            } else {
                Log.i(TAG, "Graph data was already null, no need to clear.")
            }
        }
    }

    private fun resetAllData() {
        vcount = 0
        vsum = 0.0
        steps = 0
        distance = 0.0
        v = 0.0
        maxv = 0.0
        prevxyz = 0.0
        prevt = System.nanoTime()/1000000000.0
        firstV = true
        dTime = 0.0
        prevdTime = 0.0
        prevVelocity = 0.0
        yCount = 0
        gCount = 0
        bCount = 0
        ySumPres = 0f
        gSumPres = 0f
        bSumPres = 0f
        prevPres = 0.0f
        maxPres = 1000.0f
        resetGraphData()
        stepsTextView.text = "Steps: $steps"
        distanceTextView.text = "Distance: ${0.000}km"
        velocityTextView.text = "Avg Speed: ${maxv}m/s"
        pressureTextView.text = "Toe: 0 | Ball: 0 | Heel: 0"
        updateFootColor(rightToeBackgroundView, 5100)
        updateFootColor(rightBallBackgroundView, 510)
        updateFootColor(rightHeelBackgroundView, 510)
        updatePressureText()
        Log.i(TAG, "Reset all data.")
    }

    private fun updatePressureText(){
        pressureTextView.text = "Toe: ${bSumPres/bCount} | Ball: ${ySumPres/yCount} | Heel: ${gSumPres/gCount}"
    }

    private fun setupVelocityChart(){
        Log.d(TAG, "setupVelocityChart() called.")
        velocityChart.description.isEnabled = false
        velocityChart.setTouchEnabled(true)
        velocityChart.isDragEnabled = true
        velocityChart.setScaleEnabled(true)
        velocityChart.setPinchZoom(true)
        //velocityChart.setBackgroundColor(Color.Black)

        val xAxis = velocityChart.xAxis
        xAxis.position = XAxis.XAxisPosition.BOTTOM
        xAxis.valueFormatter = object : ValueFormatter() {
            private val dateFormat = SimpleDateFormat("HH:mm:ss", Locale.getDefault())
            override fun getFormattedValue(value: Float): String { // This might be from an older IValueFormatter or a more general ValueFormatter
                return if (startTimeMillis > 0) dateFormat.format(Date(startTimeMillis + value.toLong())) else "00:00:00"
            }
        }
        xAxis.granularity = 1000f // 1 second granularity for labels
        xAxis.textColor = Color.WHITE
        xAxis.gridColor = Color.GRAY
        xAxis.setDrawGridLines(true)
        // Make sure x-axis labels are visible
        xAxis.labelRotationAngle = -45f // Rotate labels if they overlap
        xAxis.isGranularityEnabled = true


        val yAxisLeft = velocityChart.axisLeft
        yAxisLeft.textColor = Color.WHITE
        yAxisLeft.gridColor = Color.GRAY
        yAxisLeft.setDrawGridLines(true)

        yAxisLeft.axisMinimum = 0f
        yAxisLeft.axisMaximum = 5f
        yAxisLeft.labelCount = 5
        yAxisLeft.isGranularityEnabled = true
        yAxisLeft.granularity = 1f
        yAxisLeft.valueFormatter = object : ValueFormatter() {
            override fun getFormattedValue(value: Float): String {
                return "%.1fm/s".format(value)
            }
        }
        velocityChart.axisRight.isEnabled = false

        // Create an empty data set initially.
        val dataSet = LineDataSet(null, "Speed (m/s)")
        dataSet.color = Color.RED
        dataSet.valueTextColor = Color.DKGRAY
        dataSet.setCircleColor(Color.RED)
        dataSet.circleRadius = 1f
        dataSet.lineWidth = 2f

        // --- For the labels above each data point ---
        dataSet.setDrawValues(false) //false for off, true for on
        dataSet.valueTextSize = 10f
        dataSet.valueTextColor = Color.WHITE
        dataSet.valueFormatter = object : ValueFormatter() {
            override fun getFormattedValue(value: Float): String {
                return "%.1fm/s".format(value) // Formats to one decimal place and adds °C
            }
        }

        // ---------------------------------------------------

        val legend = velocityChart.legend
        legend.textColor = Color.WHITE

        val lineData = LineData(dataSet)
        velocityChart.data = lineData // IMPORTANT: Assign data object to chart
        velocityChart.invalidate() // Refresh chart
        Log.d(TAG, "Initial empty dataset assigned to chart.")
    }

    fun integrate(xf: Double, x0: Double, yf: Double, y0: Double): Double{
        Log.i(TAG, "Integrate called with xf: $xf, x0: $x0, yf: $yf, y0: $y0")
        Log.i(TAG, "Integrate should return: ${0.5*(xf+x0)*(yf-y0)}")
        return(0.500000000*(xf+x0)*(yf-y0))
    }

    fun updateVelocityDisplayAndChart(newVelocity: Float) {
        val currentTimestamp = System.currentTimeMillis()
        Log.d(TAG, "updateVelocityDisplayAndChart called with temp: $newVelocity at $currentTimestamp")

        stepsTextView.text = "Steps: ${steps}"

        // 2. Add data to the chart
        val lineData = velocityChart.data
        if (lineData != null) {
            var set = lineData.getDataSetByIndex(0) as? LineDataSet

            // If dataset doesn't exist (shouldn't happen if setup is correct)
            if (set == null) {
                Log.d(TAG, "DataSet was null, creating new one.")
                set = LineDataSet(null, "Speed (m/s)")
                set.color = Color.RED
                // ... (copy other dataset properties from velocityChart)
                set.setCircleColor(Color.RED)
                set.circleRadius = 3f
                set.lineWidth = 2f
                set.setDrawValues(false)
                lineData.addDataSet(set)
            }

            val timeElapsedMillis = currentTimestamp - startTimeMillis
            val newEntry = Entry(timeElapsedMillis.toFloat(), newVelocity)

            Log.d(TAG, "Adding entry to chart: X=${timeElapsedMillis.toFloat()}, Y=$newVelocity")
            set.addEntry(newEntry) // Add the new entry to the dataset

            // --- REMOVE OLDEST ENTRY IF MAX_VISIBLE_ENTRIES IS REACHED ---
            if (set.entryCount > MAX_VISIBLE_ENTRIES) {
                if (set.entryCount > 0) { // Safety check
                    val removed = set.removeEntry(0) // Remove entry at index 0
                    if (removed) {
                        Log.d(TAG, "Removed oldest entry from dataset to maintain MAX_VISIBLE_ENTRIES.")
                    } else {
                        Log.w(TAG, "Failed to remove oldest entry from dataset.")
                    }
                }
            }

            // Notify chart about data changes
            lineData.notifyDataChanged()
            velocityChart.notifyDataSetChanged()

            // Make the latest entry visible
            velocityChart.moveViewToX(set.entryCount.toFloat() -1) // or timeElapsedMillis.toFloat()

            velocityChart.invalidate() // Refresh the chart
            Log.d(TAG, "Chart invalidated. Entry count: ${set.entryCount}")
        } else {
            Log.e(TAG, "LineData is null! Cannot add data to chart.")
        }
    }

    private fun updateUIBasedOnConnectionStatus() {
        if(isEsp32Connected) {
            statusTextView.text = "Status: Connected to Device"
            //fillerParagraphTextView.visibility = View.VISIBLE
            connectButton.isEnabled = false
            disconnectButton.isEnabled = true
        } else {
            statusTextView.text = "Status: Disconnected from Device"
            //fillerParagraphTextView.visibility = View.GONE
            connectButton.isEnabled = true
            disconnectButton.isEnabled = false
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        disconnectFromDevice() // Ensure BLE disconnect
    }


    // --- Permission Handling ---
    private fun requestBluetoothPermissions() {
        val permissionsToRequest = mutableListOf<String>()


        // Android 12 (API 31) and above require BLUETOOTH_SCAN and BLUETOOTH_CONNECT
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
                permissionsToRequest.add(Manifest.permission.BLUETOOTH_SCAN)
            }
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
                permissionsToRequest.add(Manifest.permission.BLUETOOTH_CONNECT)
            }
        } else { // Android 11 (API 30) and below
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH) != PackageManager.PERMISSION_GRANTED) {
                permissionsToRequest.add(Manifest.permission.BLUETOOTH)
            }
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_ADMIN) != PackageManager.PERMISSION_GRANTED) {
                permissionsToRequest.add(Manifest.permission.BLUETOOTH_ADMIN)
            }
        }


        // Location permission is crucial for BLE scanning on many Android versions
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            permissionsToRequest.add(Manifest.permission.ACCESS_FINE_LOCATION)
        }


        if (permissionsToRequest.isNotEmpty()) {
            requestPermissionsLauncher.launch(permissionsToRequest.toTypedArray())
        } else {
            // Permissions already granted
            checkLocationService()
        }
    }


    // Check if Bluetooth is enabled and prompt user if not
    private fun checkBluetoothEnabled(): Boolean {
        if (bluetoothAdapter == null || !bluetoothAdapter!!.isEnabled) {
            val enableBtIntent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
            startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT)
            updateStatus("Bluetooth is OFF. Please enable it.")
            Toast.makeText(this, "Please enable Bluetooth to scan.", Toast.LENGTH_LONG).show()
            return false
        }
        return true
    }


    // Check if Location services are enabled (required for BLE scanning)
    private fun checkLocationService(): Boolean {
        val locationManager = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        if (!locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER) && !locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)) {
            Toast.makeText(this, "Location services are required for BLE scanning. Please enable them.", Toast.LENGTH_LONG).show()
            val intent = Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS)
            startActivity(intent)
            updateStatus("Location OFF. Please enable.")
            return false
        }
        return true
    }


    // Handle result of enabling Bluetooth
    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        if (requestCode == REQUEST_ENABLE_BT) {
            if (resultCode == RESULT_OK) {
                Toast.makeText(this, "Bluetooth enabled!", Toast.LENGTH_SHORT).show()
                checkLocationService() // Re-check location after Bluetooth is enabled
            } else {
                Toast.makeText(this, "Bluetooth not enabled. Cannot proceed.", Toast.LENGTH_LONG).show()
                updateStatus("Bluetooth not enabled.")
            }
        }
    }


    // --- BLE Scanning ---
    private fun startBleScan() {
        updateStatus("Scanning for Devices...")
        Log.d(TAG, "startBleScan() called.") // Log to confirm method entry



        // Pre-checks for permissions, Bluetooth, and Location
        if (!hasPermissions()) {
            requestBluetoothPermissions() // Request permissions if not granted
            updateStatus("Permissions not granted. Requesting...")
            Toast.makeText(this, "Please grant all required permissions.", Toast.LENGTH_LONG).show()
            return
        }


        if (!checkBluetoothEnabled()) { // Checks and prompts if Bluetooth is off
            return
        }


        if (!checkLocationService()) { // Checks and prompts if Location is off
            return
        }


        // If all checks pass, proceed with scan
        updateStatus("Scanning for Devices...") // Update UI immediately
        Log.i(TAG, "Starting BLE scan...") // Log scan initiation


        discoveredDevicesList.clear()
        discoveredDeviceDisplayNames.clear()
        discoveredDeviceDisplayNames.add(PROMPT_MESSAGE)
        arrayAdapter.notifyDataSetChanged()
        deviceSpinner.setSelection(0,false)

        esp32Device = null // Reset previously found device
        connectButton.isEnabled = false // Disable connect button until device is found


        // Stop any ongoing scan before starting a new one
        try {
            bluetoothLeScanner?.stopScan(bleScanCallback)
            Log.d(TAG, "Stopped any existing BLE scan.")
        } catch (e: SecurityException) {
            Log.e(TAG, "SecurityException when stopping scan: ${e.message}")
            // This error should ideally not happen if hasPermissions() is true, but good to catch
        } catch (e: Exception) {
            Log.e(TAG, "Error stopping scan: ${e.message}")
        }


        val filters: List<ScanFilter>? = null // Pass null to scan for all devices

        val scanSettings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()

        try {
            bluetoothLeScanner?.startScan(filters, scanSettings, bleScanCallback) // Pass null filters
            Log.i(TAG, "BLE scan started without service UUID filter (scanning all devices).") // Updated log

            // Stop scan after a few seconds to save power
            GlobalScope.launch(Dispatchers.Main) {
                delay(SCAN_PERIOD)
                stopBleScan()
            }
        } catch (e: SecurityException) {
            Log.e(TAG, "SecurityException when starting scan: ${e.message}")
            updateStatus("Error: Bluetooth permissions missing or location off.")
            Toast.makeText(this, "Connection failed: Please grant Bluetooth permissions and enable location.", Toast.LENGTH_LONG).show()
        } catch (e: Exception) {
            Log.e(TAG, "Unexpected error starting scan: ${e.message}")
            updateStatus("Error starting scan: ${e.message}")
        }
    }


    private fun stopBleScan() {
        Log.d(TAG, "stopBleScan() called.")
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
                    Log.e(TAG, "BLUETOOTH_SCAN permission not granted for stopBleScan.")
                }
            }
            bluetoothLeScanner?.stopScan(bleScanCallback)
            updateStatus("Scan stopped.")
            Log.i(TAG, "BLE scan successfully stopped.")
            scanButton.isEnabled = true // Re-enable scan button

            if (discoveredDevicesList.isEmpty()) {
                updateStatus("Scan stopped. No BLE devices found :(")
                Toast.makeText(this, "No BLE devices found :(", Toast.LENGTH_SHORT).show()
                connectButton.isEnabled = false
                esp32Device = null
            } else {
                updateStatus("Scan finished. Select a device from the dropdown.")
                Toast.makeText(this, "Scan finished. Found ${discoveredDevicesList.size} devices.", Toast.LENGTH_LONG).show()
            }
        } catch (e: SecurityException) {
            Log.e(TAG, "SecurityException when stopping scan: ${e.message}")
            updateStatus("Error stopping scan: Permission issue.")
            scanButton.isEnabled = true // Ensure scan button is re-enabled on error
        } catch (e: Exception) {
            Log.e(TAG, "Error stopping scan: ${e.message}")
            updateStatus("Error stopping scan.")
            scanButton.isEnabled = true // Ensure scan button is re-enabled on error
        }
    }


    private val bleScanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            super.onScanResult(callbackType, result)
            val device = result.device

            val deviceNameString = try {
                // Attempt to get the device name, handling potential SecurityException
                device.name
            } catch (e: SecurityException) {
                Log.e(TAG, "SecurityException getting device name for ${device.address}: ${e.message}")
                null // Treat as null if there's a permission issue getting the name
            }

            // Filter out devices if the name is null or just whitespace/empty
            if (deviceNameString.isNullOrBlank()) { // Use isNullOrBlank to also catch names with only whitespace
                Log.d(TAG, "Filtered out unnamed or blank-named device: ${device.address}")
                return // Skip adding this device
            }

            // Construct the displayName using the validated deviceNameString
            val displayName = "$deviceNameString - ${device.address}"

            // Check if the device (by its address) is already in actual device list
            if (!discoveredDevicesList.any { it.address == device.address }) {
                discoveredDevicesList.add(device) // Add to the actual BluetoothDevice list
                discoveredDeviceDisplayNames.add(displayName) // Add the constructed, valid displayName

                Log.d(TAG, "Device found and added: $displayName")

                // Notify the adapter that data has changed so the spinner updates
                runOnUiThread { // Ensure UI updates are on the main thread
                    arrayAdapter.notifyDataSetChanged()
                }
            }
        }

        override fun onBatchScanResults(results: MutableList<ScanResult>?) {
            super.onBatchScanResults(results)
            results?.forEach { result -> // Add null check for results
                val device = result.device
                val deviceNameString = try {
                    device.name
                } catch (e: SecurityException) {
                    Log.e(TAG, "Batch: SecurityException getting name for ${device.address}: ${e.message}")
                    null
                }

                if (deviceNameString.isNullOrBlank()) {
                    Log.d(TAG, "Filtered out unnamed or blank-named device from batch: ${device.address}")
                    return@forEach // Skip this device in the batch
                }

                val displayName = "$deviceNameString - ${device.address}"

                if (!discoveredDevicesList.any { it.address == device.address }) {
                    discoveredDevicesList.add(device)
                    discoveredDeviceDisplayNames.add(displayName)
                    Log.d(TAG, "Device found and added from batch: $displayName")
                }
            }
            // Only notify if there were actual results to process
            if (results?.isNotEmpty() == true) {
                runOnUiThread {
                    arrayAdapter.notifyDataSetChanged()
                }
            }
        }

        override fun onScanFailed(errorCode: Int) {
            super.onScanFailed(errorCode)
            Log.e(TAG, "BLE Scan Failed with error code: $errorCode")
            runOnUiThread { // Ensure UI updates are on main thread
                updateStatus("Scan failed. Error: $errorCode")
                scanButton.isEnabled = true
            }
        }
    }


    // --- BLE Connection ---
    private fun connectToDevice() {
        if (!hasPermissions()) {
            requestBluetoothPermissions()
            return
        }

        esp32Device?.let { device ->
            updateStatus("Connecting to ${device.name ?: "Unknown"}...")
            try {
                bluetoothGatt = device.connectGatt(this, false, gattCallback)
                scanButton.isEnabled = false
                connectButton.isEnabled = false
                disconnectButton.isEnabled = true
            } catch (e: SecurityException) {
                Log.e(TAG, "SecurityException when connecting to GATT: ${e.message}")
                updateStatus("Connection failed: Permissions missing.")
                Toast.makeText(this, "Connection failed: Please grant Bluetooth permissions.", Toast.LENGTH_LONG).show()
            }
        } ?: run {
            updateStatus("No device found to connect to. Please scan first.")
            Toast.makeText(this, "No Device device found. Please scan first.", Toast.LENGTH_SHORT).show()
        }
    }


    private fun disconnectFromDevice() {
        if (!hasPermissions()) {
            requestBluetoothPermissions()
            return
        }


        bluetoothGatt?.let { gatt ->
            try {
                gatt.disconnect()
                gatt.close()
                bluetoothGatt = null
                updateStatus("Disconnected from Device.")
                velocityTextView.text = "Avg Speed: N/A"
                distanceTextView.text = "Distance: N/A"
                stepsTextView.text = "Steps: N/A"
                pressureTextView.text = "Toe: 0 | Ball: 0 | Heel: 0"

                scanButton.isEnabled = true
                connectButton.isEnabled = false
                disconnectButton.isEnabled = false
            } catch (e: SecurityException) {
                Log.e(TAG, "SecurityException when disconnecting GATT: ${e.message}")
                updateStatus("Disconnection failed: Permissions missing.")
            }
        } ?: run {
            updateStatus("Not connected to any device.")
        }
    }

    private fun tryUpdateMadgwickFilter(){
        if(firstV){
            connectionTime = System.currentTimeMillis()
        }
        val currentAccel = latestAccel
        val currentGyro = latestGyro

        if(currentAccel != null && currentGyro != null) {
            currentOrientationQuaternion = madgwickFilter.update(currentOrientationQuaternion, currentGyro, currentAccel, Vector(0.0, 0.0, 0.0))
            Log.d("Madgwick", "Updated Quaternion: $currentOrientationQuaternion")
            val eulerAngles = currentOrientationQuaternion.eulerAngle
            val rollDegrees = Math.toDegrees(eulerAngles.x)
            val pitchDegrees = Math.toDegrees(eulerAngles.y)
            val yawDegrees = Math.toDegrees(eulerAngles.z)
            Log.d("Madgwick", "Roll: $rollDegrees, Pitch: $pitchDegrees, Yaw: $yawDegrees")

            val linAccel: Vector = getLinearAcceleration(currentAccel, currentOrientationQuaternion)

            //usual calcs
            val x = linAccel.x
            val y = linAccel.y
            val z = linAccel.z
            val absa: Double = sqrt(x * x + y * y + z * z)
            val timern = System.nanoTime() / 1000000000.0

            Log.d(TAG, "Current time if statement: ${System.currentTimeMillis() - connectionTime}")
            if(System.currentTimeMillis() - connectionTime > 12000) {
                calibrationText = "Calibrating IMU"
                updateStatus("Active")
                prevVelocity = v
                val tempv = integrate(absa, prevxyz, timern, prevt)
                dTime = (System.nanoTime() / 1000000000.0)
                prevxyz = absa
                prevt = timern
                var mbgang = false
                if(tempv >= 0.2) {
                    mbgang = true
                    v = tempv
                    Log.i(TAG, "speed: $v")
                    updateVelocityDisplayAndChart(v.toFloat())
                    vsum += v
                    vcount++
                } else {
                    Log.i(TAG, "speed too low: $tempv")
                    v = 0.0
                    updateVelocityDisplayAndChart(v.toFloat())
                }

                val tempDistance = integrate(v, prevVelocity, dTime, prevdTime)
                Log.i(TAG, "absa: $absa")
                if(mbgang){
                    distance+=tempDistance
                } else {
                    Log.i(TAG, "tempDistance too small: $tempDistance")
                }
                Log.i(TAG, "distance: $distance")
                distanceTextView.text = "Distance: ${((distance*1000.0*0.001).toInt())/1000.0}km"
                prevdTime = dTime

                velocityTextView.text = "Avg Speed: ${((vsum/vcount*100.0).toInt())/100.0}m/s"

                Log.i(TAG, "avg speed updated to ${vsum/vcount}")

                //first calcs
            } else {
                Log.d(TAG, "Current time else: ${((System.currentTimeMillis()/1000)).toInt()!=(dotdotdot/1000).toInt()}")

                if(((System.currentTimeMillis()/4000)).toInt() != (dotdotdot/4000).toInt() && dotdotdot != 0L){
                    dotdotdot = System.currentTimeMillis()
                    calibrationText += "."
                }

                updateStatus(calibrationText)
                prevVelocity = v
                v = integrate(absa, prevxyz, timern, prevt)
                dTime = System.nanoTime() / 1000000000.0
                prevxyz = absa
                prevt = timern
                Log.i(TAG, "first speed: $v")

                val uselessdistance = integrate(v, prevVelocity, dTime, prevdTime)
                Log.i(TAG, "first distance: $uselessdistance")
                prevdTime = dTime
                firstV = false

                Log.i(TAG, "firstV: $firstV")
                Log.d(TAG, "Current time else: ${System.currentTimeMillis() - connectionTime}")
            }
            dotdotdot = System.currentTimeMillis()
            latestAccel = null
            latestGyro = null
        }
    }


    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            super.onConnectionStateChange(gatt, status, newState)
            if (status == BluetoothGatt.GATT_SUCCESS) {
                if (newState == BluetoothProfile.STATE_CONNECTED) {
                    Log.i(TAG, "Connected to GATT client. Discovering services...")
                    currentGatt = gatt // Store the gatt instance
                    runOnUiThread {
                        updateStatus("Connected to Device. Discovering services...")
                        isEsp32Connected = true
                        connectButton.isEnabled = false
                        disconnectButton.isEnabled = true
                        updateUIBasedOnConnectionStatus()
                    }
                    try {
                        val discoverServicesSuccess = gatt.discoverServices()
                        if (!discoverServicesSuccess) {
                            Log.e(TAG, "discoverServices() returned false")
                            // Handle this error - perhaps disconnect and try again
                        }
                    } catch (e: SecurityException) {
                        Log.e(TAG, "SecurityException when discovering services: ${e.message}")
                        runOnUiThread { updateStatus("Error discovering services: Permissions missing.") }
                    }
                } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                    Log.i(TAG, "Disconnected from GATT client.")
                    runOnUiThread {
                        updateStatus("Disconnected from Device.")
                        velocityTextView.text = "Avg Speed: N/A"
                        distanceTextView.text = "Distance: N/A"
                        stepsTextView.text = "Steps: N/A"
                        pressureTextView.text = "Toe: 0 | Ball: 0 | Heel: 0"
                        scanButton.isEnabled = true
                        connectButton.isEnabled = false
                        disconnectButton.isEnabled = false
                        bluetoothGatt?.close()
                        bluetoothGatt = null
                        isEsp32Connected = false
                        updateUIBasedOnConnectionStatus()
                    }
                }
            } else {
                Log.e(TAG, "Connection state change error: $status")
                runOnUiThread {
                    updateStatus("Connection error: $status")
                    scanButton.isEnabled = true
                    connectButton.isEnabled = false
                    disconnectButton.isEnabled = false
                    currentGatt?.close() // Use currentGatt here
                    currentGatt = null
                    characteristicSubscriptionQueue.clear() // Clear queue on error
                }
            }
        }


        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            super.onServicesDiscovered(gatt, status)
            Log.i(TAG, "onServicesDiscovered received status: $status")

            if (status == BluetoothGatt.GATT_SUCCESS) {
                val service = gatt.getService(SERVICE_UUID)
                if (service != null) {
                    Log.i(TAG, "Service ${SERVICE_UUID} found.")
                    characteristicSubscriptionQueue.clear() // Clear before adding new ones

                    // --- PRESSURE ---
                    val ypressureChar = service.getCharacteristic(YPRESSURE_CHAR_UUID)
                    if (ypressureChar != null) {
                        Log.i(TAG, "Pressure characteristic ${YPRESSURE_CHAR_UUID} found. Adding to queue.")
                        characteristicSubscriptionQueue.offer(ypressureChar)
                    } else {
                        Log.e(TAG, "Pressure characteristic ${YPRESSURE_CHAR_UUID} NOT FOUND.")
                    }
                    val gpressureChar = service.getCharacteristic(GPRESSURE_CHAR_UUID)
                    if (gpressureChar != null) {
                        Log.i(TAG, "Pressure characteristic ${GPRESSURE_CHAR_UUID} found. Adding to queue.")
                        characteristicSubscriptionQueue.offer(gpressureChar)
                    } else {
                        Log.e(TAG, "Pressure characteristic ${GPRESSURE_CHAR_UUID} NOT FOUND.")
                    }
                    val bpressureChar = service.getCharacteristic(BPRESSURE_CHAR_UUID)
                    if (bpressureChar != null) {
                        Log.i(TAG, "Pressure characteristic ${BPRESSURE_CHAR_UUID} found. Adding to queue.")
                        characteristicSubscriptionQueue.offer(bpressureChar)
                    } else {
                        Log.e(TAG, "Pressure characteristic ${GPRESSURE_CHAR_UUID} NOT FOUND.")
                    }


                    // --- ACCEL ---
                    Log.d(TAG, "--- Finding ACCEL ---") // Just finding, not processing yet
                    val AccelChar = service.getCharacteristic(ACCEL_CHAR_UUID)
                    if (AccelChar != null) {
                        Log.i(TAG, "Acceleration characteristic ${ACCEL_CHAR_UUID} found. Adding to queue.")
                        characteristicSubscriptionQueue.offer(AccelChar)
                    } else {
                        Log.e(TAG, "Acceleration characteristic ${ACCEL_CHAR_UUID} NOT FOUND.")
                    }
                    // --- ROT ---
                    Log.d(TAG, "--- Finding ROT ---") // Just finding, not processing yet
                    val RotChar = service.getCharacteristic(ROT_CHAR_UUID)
                    if (RotChar != null) {
                        Log.i(TAG, "Rotation characteristic ${ROT_CHAR_UUID} found. Adding to queue.")
                        characteristicSubscriptionQueue.offer(RotChar)
                    } else {
                        Log.e(TAG, "Rotation characteristic ${ROT_CHAR_UUID} NOT FOUND.")
                    }
                    runOnUiThread { updateStatus("Service discovery complete. Starting subscriptions.") }
                    connectionTime = System.currentTimeMillis()
                    Log.d(TAG, "onDescriptorWrite: About to call processNextCharacteristicSubscription. Queue size: ${characteristicSubscriptionQueue.size}")
                    processNextCharacteristicSubscription() // THIS WILL HANDLE THE ACTUAL SUBSCRIPTIONS

                } else {
                    Log.e(TAG, "Service ${SERVICE_UUID} NOT FOUND.")
                    runOnUiThread { updateStatus("Error: Service not found.") }
                }
            } else {
                Log.e(TAG, "onServicesDiscovered received failure status: $status")
                runOnUiThread { updateStatus("Error discovering services: $status") }
            }
        }

        private fun processNextCharacteristicSubscription() {
            Log.d(TAG, "processNextCharacteristicSubscription: Entered. Queue size: ${characteristicSubscriptionQueue.size}")
            if (characteristicSubscriptionQueue.isEmpty()) {
                Log.i(TAG, "All characteristics in queue processed for notifications.")
                runOnUiThread { updateStatus("Characteristic subscriptions complete.") }
                return
            }

            val characteristic = characteristicSubscriptionQueue.peek()
            if (characteristic == null) {
                Log.w(TAG, "Queue peeked null, something is wrong or queue became empty concurrently.")
                characteristicSubscriptionQueue.poll()
                processNextCharacteristicSubscription()
                return
            }


            Log.d(TAG, "Processing subscription for: ${characteristic.uuid}")

            try {
                if (currentGatt == null) {
                    Log.e(TAG, "GATT is null, cannot subscribe to ${characteristic.uuid}")
                    characteristicSubscriptionQueue.clear() // Cannot proceed
                    return
                }

                // 1. Enable notifications locally
                val notificationSet = currentGatt!!.setCharacteristicNotification(characteristic, true)
                Log.d(TAG, "${characteristic.uuid}: setCharacteristicNotification returned: $notificationSet")

                if (notificationSet) {
                    // 2. Get the CCCD (Client Characteristic Configuration Descriptor)
                    val descriptor = characteristic.getDescriptor(CLIENT_CHARACTERISTIC_CONFIG_UUID)
                    if (descriptor != null) {
                        Log.d(TAG, "${characteristic.uuid}: CCCD ${CLIENT_CHARACTERISTIC_CONFIG_UUID} found.")
                        descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                        // 3. Write to the descriptor on the peripheral
                        Log.d(TAG, "${characteristic.uuid}: Attempting writeDescriptor")
                        val writeSuccess = currentGatt!!.writeDescriptor(descriptor)
                        Log.d(TAG, "${characteristic.uuid}: writeDescriptor initiated: $writeSuccess")
                        if (!writeSuccess) {
                            Log.e(TAG, "${characteristic.uuid}: writeDescriptor call failed immediately. Skipping this characteristic.")
                            characteristicSubscriptionQueue.poll() // Remove current from queue
                            processNextCharacteristicSubscription() // Try next
                        }
                        // If writeSuccess is true, wait for onDescriptorWrite callback
                    } else {
                        Log.e(TAG, "${characteristic.uuid}: CCCD ${CLIENT_CHARACTERISTIC_CONFIG_UUID} NOT FOUND! Skipping.")
                        characteristicSubscriptionQueue.poll() // Remove current from queue
                        processNextCharacteristicSubscription() // Try next
                    }
                } else {
                    Log.e(TAG, "${characteristic.uuid}: setCharacteristicNotification failed. Skipping.")
                    characteristicSubscriptionQueue.poll() // Remove current from queue
                    processNextCharacteristicSubscription() // Try next
                }
            } catch (e: SecurityException) {
                Log.e(TAG, "${characteristic.uuid}: SecurityException during subscription: ${e.message}", e)
                characteristicSubscriptionQueue.poll() // Remove current from queue
                processNextCharacteristicSubscription() // Try next
            } catch (e: Exception) {
                Log.e(TAG, "${characteristic.uuid}: Generic Exception during subscription: ${e.message}", e)
                characteristicSubscriptionQueue.poll() // Remove current from queue
                processNextCharacteristicSubscription() // Try next
            }
        }


        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
            super.onCharacteristicChanged(gatt, characteristic)

            val rawValue = characteristic.getStringValue(0) ?: ""
            Log.d(TAG, "Characteristic ${characteristic.uuid} changed. Raw String Value: '$rawValue'")

            runOnUiThread {


                when (characteristic.uuid) { // characteristic is also captured

                    YPRESSURE_CHAR_UUID -> {

                        val pressureString = rawValue.replace("g", "").trim()

                        try {
                            val pressureFloat = pressureString.toFloat()
                            val stepsConstant = 150
                            if(prevPres <= stepsConstant && pressureFloat > stepsConstant || prevPres >= stepsConstant && pressureFloat < stepsConstant) {
                                steps++
                                stepsTextView.text = "Steps: $steps"
                                prevPres = pressureFloat
                            }
                            Log.i(TAG, "ypressureFloat: $pressureFloat")
                            ySumPres += pressureFloat
                            yCount++
                            if(pressureFloat > 0f){
                                Log.d(TAG, "ySumPres: $ySumPres, yCount: $yCount")
                                val avgPres: Float = (ySumPres/yCount)
                                updateFootColor(rightBallBackgroundView, ((510-((avgPres/maxPres)*510)).toInt()))
                            }
                            updatePressureText()
                        } catch (e: NumberFormatException) {
                            Log.e(TAG, "Could not parse pressure string '$pressureString' to Float.", e)
                        }
                    }

                    GPRESSURE_CHAR_UUID -> {

                        val pressureString = rawValue.replace("g", "").trim()

                        try {
                            val pressureFloat = pressureString.toFloat()
                            gSumPres += pressureFloat
                            gCount++
                            Log.i(TAG, "gAvgPres: ${((gSumPres/gCount)).toInt()} gsum: $gSumPres, gCount: $gCount")
                            if(pressureFloat > 0f){
                                Log.d(TAG, "gSumPres: $gSumPres, gCount: $gCount")
                                val avgPres: Float = (gSumPres/gCount)
                                updateFootColor(rightHeelBackgroundView, ((510-((avgPres/maxPres)*510)).toInt()))
                            }
                            updatePressureText()
                        } catch (e: NumberFormatException) {
                            Log.e(TAG, "Could not parse pressure string '$pressureString' to Float.", e)
                        }
                    }

                    BPRESSURE_CHAR_UUID -> {

                        val pressureString = rawValue.replace("g", "").trim()


                        try {
                            val pressureFloat = pressureString.toFloat()
                            bSumPres += pressureFloat
                            bCount++
                            if(pressureFloat > 0f){
                                Log.d(TAG, "bSumPres: $bSumPres, bCount: $bCount")
                                val avgPres: Float = (bSumPres/bCount)
                                updateFootColor(rightToeBackgroundView, ((510-((avgPres/maxPres)*510)).toInt()))
                            }
                            updatePressureText()
                        } catch (e: NumberFormatException) {
                            Log.e(TAG, "Could not parse pressure string '$pressureString' to Float.", e)
                        }
                    }

                    ACCEL_CHAR_UUID -> {
                        val rawDataString = characteristic.getStringValue(0)
                        Log.i(TAG, "Received Accel Data: '$rawValue'")

                        if(!rawDataString.isNullOrEmpty()) {
                            try {
                                val matchResult = accelDataRegex.find(rawDataString)
                                if(matchResult != null) {
                                    val (xStr, yStr, zStr) = matchResult.destructured
                                    try{
                                        val ax = xStr.toFloat()
                                        val ay = yStr.toFloat()
                                        val az = zStr.toFloat()
                                        Log.i(TAG, "ax: $ax, ay: $ay, az: $az")

                                        latestAccel = Vector(ax.toDouble()*GRAVITY_MAGNITUDE, ay.toDouble()*GRAVITY_MAGNITUDE, az.toDouble()*GRAVITY_MAGNITUDE)



                                    } catch (e: NumberFormatException){
                                        Log.e(TAG, "Could not parse acceleration string: '$rawValue'", e)
                                    }
                                }

                            } catch (e: NumberFormatException) {
                                Log.e(TAG, "Could not parse acceleration string: '$rawValue'", e)
                            }
                        } else {
                            Log.e(TAG, "Received null acceleration data.")
                        }
                    }
                    ROT_CHAR_UUID -> {
                        val rawDataString = characteristic.getStringValue(0)
                        Log.i(TAG, "Received Rot Data: '$rawValue'")

                        if(!rawDataString.isNullOrEmpty()) {
                            try {
                                val matchResult = rotDataRegex.find(rawDataString)
                                if(matchResult != null) {
                                    val(xxStr, yyStr, zzStr) = matchResult.destructured
                                    try{
                                        val gx = xxStr.toFloat()
                                        val gy = yyStr.toFloat()
                                        val gz = zzStr.toFloat()
                                        Log.i(TAG, "xx: $gx, yy: $gy, zz: $gz")

                                        latestGyro = Vector(gx.toDouble(), gy.toDouble(), gz.toDouble())
                                        tryUpdateMadgwickFilter()
                                    } catch (e: NumberFormatException){
                                        Log.e(TAG, "Could not parse rotation string: '$rawValue'", e)
                                    }
                                }
                            } catch (e: NumberFormatException) {
                                Log.e(TAG, "Could not parse rotation string: '$rawValue'", e)
                            }
                        } else {
                            Log.e(TAG, "Received null rotation data.")
                        }
                    }
                }
            }
        }

        override fun onDescriptorWrite(gatt: BluetoothGatt?, descriptor: BluetoothGattDescriptor?, status: Int) {
            super.onDescriptorWrite(gatt, descriptor, status)

            val writtenCharacteristic = characteristicSubscriptionQueue.poll() // Remove the characteristics just processed

            if (writtenCharacteristic == null) {
                Log.w(TAG, "onDescriptorWrite: Polled null from queue. Descriptor UUID: ${descriptor?.uuid ?: "N/A descriptor was null"}, Status: $status")

            } else {
                val charUuid = writtenCharacteristic.uuid
                Log.d(TAG, "onDescriptorWrite for characteristic $charUuid. Descriptor UUID: ${descriptor?.uuid}, Status: $status")
                if (descriptor == null) { // descriptor.characteristic could also be null
                    Log.e(TAG, "onDescriptorWrite: Descriptor is null for characteristic $charUuid! Status: $status")

                } else {
                }
                if (status == BluetoothGatt.GATT_SUCCESS) {
                    Log.i(TAG, "SUCCESS: Wrote descriptor for Characteristic UUID: $charUuid. Notifications should be enabled.")
                    runOnUiThread {
                        when (charUuid) {
                            GPRESSURE_CHAR_UUID -> updateStatus("Subscribed to Pressure.")
                            ACCEL_CHAR_UUID -> updateStatus("Subscribed to X Accel.")
                            //Y_ACCEL_CHAR_UUID -> updateStatus("Subscribed to Y Accel.")
                            else -> updateStatus("Subscribed to unknown characteristic: $charUuid")
                        }
                        if (characteristicSubscriptionQueue.isEmpty()) {
                            Log.i(TAG, "All characteristic subscriptions complete.")
                        }
                    }
                } else {
                    Log.e(TAG, "FAILURE: Failed to write descriptor for Characteristic UUID: $charUuid, Status: $status")
                    runOnUiThread {
                        updateStatus("Failed to subscribe to notifications for $charUuid (Error: $status)")
                    }
                }
            }
            Log.d(TAG, "onDescriptorWrite: About to call processNextCharacteristicSubscription. Queue size: ${characteristicSubscriptionQueue.size}")
            processNextCharacteristicSubscription()
        }
    }

    // --- Utility Functions ---
    private fun updateStatus(message: String) {
        runOnUiThread {
            statusTextView.text = "Status: $message"
        }
    }

    private fun hasPermissions(): Boolean {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) == PackageManager.PERMISSION_GRANTED &&
                    ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED &&
                    ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
        } else {
            ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH) == PackageManager.PERMISSION_GRANTED &&
                    ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_ADMIN) == PackageManager.PERMISSION_GRANTED &&
                    ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
        }
    }

    companion object {
        private const val TAG = "SNEAKSYNC"
        private const val REQUEST_ENABLE_BT = 1
        private const val SCAN_PERIOD: Long = 2000 // Scan for 2 seconds
    }
}