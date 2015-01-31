package com.moov2.spacegun.gun;

import android.app.Activity;
import android.app.Fragment;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.net.Uri;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import com.moov2.spacegun.R;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * A simple {@link Fragment} subclass.
 * Activities that contain this fragment must implement the
 * {@link GunFragment.OnFragmentInteractionListener} interface
 * to handle interaction events.
 * Use the {@link GunFragment#newInstance} factory method to
 * create an instance of this fragment.
 */
public class GunFragment extends Fragment implements SensorEventListener {
    // TODO: Rename parameter arguments, choose names that match
    // the fragment initialization parameters, e.g. ARG_ITEM_NUMBER
    private static final String ARG_PARAM1 = "param1";
    private static final String ARG_PARAM2 = "param2";


    // Calibrated maths.
    private float[] currentRotationMatrixCalibrated;
    private float[] deltaRotationMatrixCalibrated;
    private float[] deltaRotationVectorCalibrated;
    private float[] gyroscopeOrientationCalibrated;

    // Uncalibrated maths
    private float[] currentRotationMatrixRaw;
    private float[] deltaRotationMatrixRaw;
    private float[] deltaRotationVectorRaw;
    private float[] gyroscopeOrientationRaw;

    private DecimalFormat df;

    // accelerometer and magnetometer based rotation matrix
    private float[] initialRotationMatrix;
    // TODO: Rename and change types of parameters
    private SensorManager mSensorManager;
    private TextView mData1;
    private TextView mData2;
    private TextView mData3;
    private static final int MIN_SAMPLE_COUNT = 30;
    private static final int MEAN_FILTER_WINDOW = 10;

    private boolean hasInitialOrientation = false;
    private boolean stateInitializedCalibrated = false;
    private boolean stateInitializedRaw = false;

    private MeanFilter accelerationFilter;
    private MeanFilter magneticFilter;

    // accelerometer vector
    private float[] acceleration;

    // magnetic field vector
    private float[] magnetic;

    private int accelerationSampleCount = 0;
    private int magneticSampleCount = 0;

    private long timestampOldCalibrated = 0;
    private long timestampOldRaw = 0;

    private OnFragmentInteractionListener mListener;

    /**
     * Use this factory method to create a new instance of
     * this fragment using the provided parameters.
     *
     * @param param1 Parameter 1.
     * @param param2 Parameter 2.
     * @return A new instance of fragment GunFragment.
     */
    // TODO: Rename and change types and number of parameters
    public static GunFragment newInstance(String param1, String param2) {
        GunFragment fragment = new GunFragment();
        Bundle args = new Bundle();
        fragment.setArguments(args);
        return fragment;
    }

    public GunFragment() {
        // Required empty public constructor
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (getArguments() != null) {
        }
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        return inflater.inflate(R.layout.fragment_gun, null);
    }

    // TODO: Rename method, update argument and hook method into UI event
    public void onButtonPressed(Uri uri) {
        if (mListener != null) {
            mListener.onFragmentInteraction(uri);
        }
    }

    @Override
    public void onViewCreated(View view, Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        mSensorManager = (SensorManager) getActivity().getSystemService(Context.SENSOR_SERVICE);
        initUI();
        initMaths();
        initSensors();
        initFilters();
    }

    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
        try {
            mListener = (OnFragmentInteractionListener) activity;
        } catch (ClassCastException e) {
            throw new ClassCastException(activity.toString()
                    + " must implement OnFragmentInteractionListener");
        }
    }

    @Override
    public void onDetach() {
        super.onDetach();
        mListener = null;
    }

    @Override
    public void onResume() {
        super.onResume();
        restart();
    }

    @Override
    public void onPause() {
        super.onPause();
        reset();
    }

    private static final float NS2S = 1.0f / 1000000000.0f;
    private final float[] deltaRotationVector = new float[4];
    private float timestamp;
    private static final float EPSILON = 0;
    private final float[] currentRotation = new float[9];

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            onAccelerationSensorChanged(event.values, event.timestamp);
        }

        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            onMagneticSensorChanged(event.values, event.timestamp);
        }

        if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            onGyroscopeSensorChanged(event.values, event.timestamp);
        }
    }

    private void onAccelerationSensorChanged(float[] acceleration, long timestamp) {
// Get a local copy of the raw magnetic values from the device sensor.
        System.arraycopy(acceleration, 0, this.acceleration, 0,
                acceleration.length);

        // Use a mean filter to smooth the sensor inputs
        this.acceleration = accelerationFilter.filterFloat(this.acceleration);

        // Count the number of samples received.
        accelerationSampleCount++;

        // Only determine the initial orientation after the acceleration sensor
        // and magnetic sensor have had enough time to be smoothed by the mean
        // filters. Also, only do this if the orientation hasn't already been
        // determined since we only need it once.
        if (accelerationSampleCount > MIN_SAMPLE_COUNT
                && magneticSampleCount > MIN_SAMPLE_COUNT
                && !hasInitialOrientation) {
            calculateOrientation();
        }
    }

    private void onMagneticSensorChanged(float[] magnetic, long timestamp) {
// Get a local copy of the raw magnetic values from the device sensor.
        System.arraycopy(magnetic, 0, this.magnetic, 0, magnetic.length);

        // Use a mean filter to smooth the sensor inputs
        this.magnetic = magneticFilter.filterFloat(this.magnetic);

        // Count the number of samples received.
        magneticSampleCount++;
    }

    private void onGyroscopeSensorChanged(float[] gyroscope, long timestamp) {
        // don't start until first accelerometer/magnetometer orientation has
        // been acquired
        if (!hasInitialOrientation) {
            return;
        }

        // Initialization of the gyroscope based rotation matrix
        if (!stateInitializedCalibrated) {
            currentRotationMatrixCalibrated = matrixMultiplication(
                    currentRotationMatrixCalibrated, initialRotationMatrix);

            stateInitializedCalibrated = true;
        }

        // This timestep's delta rotation to be multiplied by the current
        // rotation after computing it from the gyro sample data.
        if (timestampOldCalibrated != 0 && stateInitializedCalibrated) {
            final float dT = (timestamp - timestampOldCalibrated) * NS2S;

            // Axis of the rotation sample, not normalized yet.
            float axisX = gyroscope[0];
            float axisY = gyroscope[1];
            float axisZ = gyroscope[2];

            // Calculate the angular speed of the sample
            float omegaMagnitude = (float) Math.sqrt(axisX * axisX + axisY
                    * axisY + axisZ * axisZ);

            // Normalize the rotation vector if it's big enough to get the axis
            if (omegaMagnitude > EPSILON) {
                axisX /= omegaMagnitude;
                axisY /= omegaMagnitude;
                axisZ /= omegaMagnitude;
            }

            // Integrate around this axis with the angular speed by the timestep
            // in order to get a delta rotation from this sample over the
            // timestep. We will convert this axis-angle representation of the
            // delta rotation into a quaternion before turning it into the
            // rotation matrix.
            float thetaOverTwo = omegaMagnitude * dT / 2.0f;

            float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
            float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);

            deltaRotationVectorCalibrated[0] = sinThetaOverTwo * axisX;
            deltaRotationVectorCalibrated[1] = sinThetaOverTwo * axisY;
            deltaRotationVectorCalibrated[2] = sinThetaOverTwo * axisZ;
            deltaRotationVectorCalibrated[3] = cosThetaOverTwo;

            SensorManager.getRotationMatrixFromVector(
                    deltaRotationMatrixCalibrated,
                    deltaRotationVectorCalibrated);

            currentRotationMatrixCalibrated = matrixMultiplication(
                    currentRotationMatrixCalibrated,
                    deltaRotationMatrixCalibrated);

            SensorManager.getOrientation(currentRotationMatrixCalibrated,
                    gyroscopeOrientationCalibrated);
        }

        timestampOldCalibrated = timestamp;


        mData1.setText(df.format(Math
                .toDegrees(gyroscopeOrientationCalibrated[0])));
        mData2.setText(df.format(Math
                .toDegrees(gyroscopeOrientationCalibrated[1])));
        mData3.setText(df.format(Math
                .toDegrees(gyroscopeOrientationCalibrated[2])));
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    /**
     * This interface must be implemented by activities that contain this
     * fragment to allow an interaction in this fragment to be communicated
     * to the activity and potentially other fragments contained in that
     * activity.
     * <p/>
     * See the Android Training lesson <a href=
     * "http://developer.android.com/training/basics/fragments/communicating.html"
     * >Communicating with Other Fragments</a> for more information.
     */
    public interface OnFragmentInteractionListener {
        // TODO: Update argument type and name
        public void onFragmentInteraction(Uri uri);
    }

    private void reset() {
        mSensorManager.unregisterListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER));

        mSensorManager.unregisterListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD));

        mSensorManager.unregisterListener(this, mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE));

        initMaths();

        accelerationSampleCount = 0;
        magneticSampleCount = 0;

        hasInitialOrientation = false;
        stateInitializedCalibrated = false;
        stateInitializedRaw = false;
    }

    private void restart() {
        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_FASTEST);

        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                SensorManager.SENSOR_DELAY_FASTEST);

        // Do not register for gyroscope updates if we are going to use the
        // fused version of the sensor...
        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                SensorManager.SENSOR_DELAY_FASTEST);
    }

    /**
     * Initialize the UI.
     */
    private void initUI() {
        // Get a decimal formatter for the text views
        df = new DecimalFormat("#.##");


        mData1 = (TextView) getView().findViewById(R.id.data1);
        mData2 = (TextView) getView().findViewById(R.id.data2);
        mData3 = (TextView) getView().findViewById(R.id.data3);
    }

    /**
     * Initialize the data structures required for the maths.
     */
    private void initMaths() {
        acceleration = new float[3];
        magnetic = new float[3];

        initialRotationMatrix = new float[9];

        deltaRotationVectorCalibrated = new float[4];
        deltaRotationMatrixCalibrated = new float[9];
        currentRotationMatrixCalibrated = new float[9];
        gyroscopeOrientationCalibrated = new float[3];

        // Initialize the current rotation matrix as an identity matrix...
        currentRotationMatrixCalibrated[0] = 1.0f;
        currentRotationMatrixCalibrated[4] = 1.0f;
        currentRotationMatrixCalibrated[8] = 1.0f;

        deltaRotationVectorRaw = new float[4];
        deltaRotationMatrixRaw = new float[9];
        currentRotationMatrixRaw = new float[9];
        gyroscopeOrientationRaw = new float[3];

        // Initialize the current rotation matrix as an identity matrix...
        currentRotationMatrixRaw[0] = 1.0f;
        currentRotationMatrixRaw[4] = 1.0f;
        currentRotationMatrixRaw[8] = 1.0f;
    }

    /**
     * Initialize the sensors.
     */
    private void initSensors() {
        mSensorManager = (SensorManager) getActivity()
                .getSystemService(Context.SENSOR_SERVICE);
    }

    /**
     * Initialize the mean filters.
     */
    private void initFilters() {
        accelerationFilter = new MeanFilter();
        accelerationFilter.setWindowSize(MEAN_FILTER_WINDOW);

        magneticFilter = new MeanFilter();
        magneticFilter.setWindowSize(MEAN_FILTER_WINDOW);
    }

    /**
     * Calculates orientation angles from accelerometer and magnetometer output.
     * Note that we only use this *once* at the beginning to orient the
     * gyroscope to earth frame. If you do not call this, the gyroscope will
     * orient itself to whatever the relative orientation the device is in at
     * the time of initialization.
     */
    private void calculateOrientation() {
        hasInitialOrientation = SensorManager.getRotationMatrix(
                initialRotationMatrix, null, acceleration, magnetic);

        // Remove the sensor observers since they are no longer required.
        if (hasInitialOrientation) {
            mSensorManager.unregisterListener(this,
                    mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER));
            mSensorManager.unregisterListener(this,
                    mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD));
        }
    }

    /**
     * Multiply matrix a by b. Android gives us matrices results in
     * one-dimensional arrays instead of two, so instead of using some (O)2 to
     * transfer to a two-dimensional array and then an (O)3 algorithm to
     * multiply, we just use a static linear time method.
     *
     * @param a
     * @param b
     * @return a*b
     */
    private float[] matrixMultiplication(float[] a, float[] b) {
        float[] result = new float[9];

        result[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
        result[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
        result[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];

        result[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
        result[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
        result[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];

        result[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
        result[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
        result[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];

        return result;
    }

    /**
     * Implements a mean filter designed to smooth the data points based on a mean.
     *
     * @author Kaleb
     * @version %I%, %G%
     */
    public class MeanFilter {
        // The size of the mean filters rolling window.
        private int filterWindow = 30;

        private boolean dataInit;

        private ArrayList<LinkedList<Number>> dataLists;

        /**
         * Initialize a new MeanFilter object.
         */
        public MeanFilter() {
            dataLists = new ArrayList<LinkedList<Number>>();
            dataInit = false;
        }

        /**
         * Filter the data.
         * <p/>
         * contains input the data.
         *
         * @return the filtered output data.
         */
        public float[] filterFloat(float[] data) {
            for (int i = 0; i < data.length; i++) {
                // Initialize the data structures for the data set.
                if (!dataInit) {
                    dataLists.add(new LinkedList<Number>());
                }

                dataLists.get(i).addLast(data[i]);

                if (dataLists.get(i).size() > filterWindow) {
                    dataLists.get(i).removeFirst();
                }
            }

            dataInit = true;

            float[] means = new float[dataLists.size()];

            for (int i = 0; i < dataLists.size(); i++) {
                means[i] = (float) getMean(dataLists.get(i));
            }

            return means;
        }

        /**
         * Get the mean of the data set.
         *
         * @param data the data set.
         * @return the mean of the data set.
         */
        private float getMean(List<Number> data) {
            float m = 0;
            float count = 0;

            for (int i = 0; i < data.size(); i++) {
                m += data.get(i).floatValue();
                count++;
            }

            if (count != 0) {
                m = m / count;
            }

            return m;
        }

        public void setWindowSize(int size) {
            this.filterWindow = size;
        }
    }
}
