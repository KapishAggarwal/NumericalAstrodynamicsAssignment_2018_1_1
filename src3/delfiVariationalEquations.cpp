/*    Copyright (c) 2010-2017, Delft University of Technology
 *    All rigths reserved
 *
 *    This file is part of the Tudat. Redistribution and use in source and
 *    binary forms, with or without modification, are permitted exclusively
 *    under the terms of the Modified BSD license. You should have received
 *    a copy of the license with this file. If not, please or visit:
 *    http://tudat.tudelft.nl/LICENSE.
 */

#include <Tudat/SimulationSetup/tudatEstimationHeader.h>

//! Get path for output directory.
static inline std::string getOutputPath(
        const std::string& extraDirectory = "" )
{
    // Declare file path string assigned to filePath.
    // __FILE__ only gives the absolute path of the header file!
    std::string filePath_( __FILE__ );

    // Strip filename from temporary string and return root-path string.
    std::string reducedPath = filePath_.substr( 0, filePath_.length( ) -
                                                std::string( "delfiVariationalEquations.cpp" ).length( ) );
    std::string outputPath = reducedPath + "SimulationOutput/";
    if( extraDirectory != "" )
    {
        outputPath += extraDirectory;
    }

    if( outputPath.at( outputPath.size( ) - 1 ) != '/' )
    {
        outputPath += "/";
    }

    return outputPath;
}

//! Execute propagation of orbit of Delfi around the Earth.
int main()
{
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////            USING STATEMENTS              //////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    using namespace tudat;
    using namespace tudat::simulation_setup;
    using namespace tudat::propagators;
    using namespace tudat::numerical_integrators;
    using namespace tudat::orbital_element_conversions;
    using namespace tudat::basic_mathematics;
    using namespace tudat::gravitation;
    using namespace tudat::numerical_integrators;
    using namespace tudat::estimatable_parameters;
    using namespace tudat::ephemerides;


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////     CREATE ENVIRONMENT AND VEHICLE       //////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // Load Spice kernels.
    spice_interface::loadStandardSpiceKernels( );

    // Set simulation time settings.
    double simulationStartEpoch = tudat::physical_constants::JULIAN_YEAR;

    // **** MODIFY FOR AE4868: define list of bodies required for simulation
    double simulationEndEpoch;// = ...

    // Define body settings for simulation.
    std::vector< std::string > bodiesToCreate;

    // Set Earth gravity field settings.
    std::map< std::string, std::shared_ptr< BodySettings > > bodySettings =
            getDefaultBodySettings( bodiesToCreate, simulationStartEpoch - 300.0, simulationEndEpoch + 300.0 );
    bodySettings[ "Earth" ]->gravityFieldSettings =
            std::make_shared< FromFileSphericalHarmonicsGravityFieldSettings >( ggm02c );
    for( unsigned int i = 0; i < bodiesToCreate.size( ); i++ )
    {
        bodySettings[ bodiesToCreate.at( i ) ]->ephemerisSettings->resetFrameOrientation( "J2000" );
        bodySettings[ bodiesToCreate.at( i ) ]->rotationModelSettings->resetOriginalFrame( "J2000" );
    }

    // Create body objects.
    NamedBodyMap bodyMap = createBodies( bodySettings );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////             CREATE VEHICLE            /////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Create spacecraft object.
    bodyMap[ "Delfi" ] = std::make_shared< simulation_setup::Body >( );
    bodyMap[ "Delfi" ]->setEphemeris( std::make_shared< TabulatedCartesianEphemeris< > >(
                                          std::shared_ptr< interpolators::OneDimensionalInterpolator
                                          < double, Eigen::Vector6d > >( ), "Earth", "J2000" ) );

    // **** MODIFY FOR AE4868: all required properties of body Delfi
    bodyMap[ "Delfi" ]->setConstantBodyMass( 3.5 );

       bodyMap[ "Delfi" ]->setEphemeris( boost::make_shared< TabulatedCartesianEphemeris< > >(
                                             boost::shared_ptr< interpolators::OneDimensionalInterpolator
                                             < double, Eigen::Vector6d > >( ), "Earth", "J2000" ) );

       // Create aerodynamic coefficient interface settings.
       double referenceArea = 0.02;
       double aerodynamicCoefficient = 1.5;
       boost::shared_ptr< AerodynamicCoefficientSettings > aerodynamicCoefficientSettings =
               boost::make_shared< ConstantAerodynamicCoefficientSettings >(
                   referenceArea, aerodynamicCoefficient * Eigen::Vector3d::UnitX( ), 1, 1 );

       // Create and set aerodynamic coefficients object
       bodyMap[ "Delfi" ]->setAerodynamicCoefficientInterface(
                   createAerodynamicCoefficientInterface( aerodynamicCoefficientSettings, "Delfi" ) );

       // Create radiation pressure settings
       double referenceAreaRadiation = 0.02;
       double radiationPressureCoefficient = 1.2;
       std::vector< std::string > occultingBodies;
       occultingBodies.push_back( "Earth" );
       boost::shared_ptr< RadiationPressureInterfaceSettings > delfiRadiationPressureSettings =
               boost::make_shared< CannonBallRadiationPressureInterfaceSettings >(
                   "Sun", referenceAreaRadiation, radiationPressureCoefficient, occultingBodies );

       // Create and set radiation pressure settings
       bodyMap[ "Delfi" ]->setRadiationPressureInterface(
                   "Sun", createRadiationPressureInterface(
                       delfiRadiationPressureSettings, "Delfi", bodyMap ) );

    // Finalize body creation.
    setGlobalFrameBodyEphemerides( bodyMap, "SSB", "J2000" );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////            CREATE ACCELERATIONS          //////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Define propagator settings variables.
    std::vector< std::string > bodiesToPropagate;
    bodiesToPropagate.push_back( "Delfi" );
    std::vector< std::string > centralBodies;
    centralBodies.push_back( "Earth" );

    // **** MODIFY FOR AE4868: define settings for accelerations acting on Delfi.
    std::map< std::string, std::vector< std::shared_ptr< AccelerationSettings > > > accelerationsOfDelfi;


    SelectedAccelerationMap accelerationMap;
    accelerationMap[ "Delfi" ] = accelerationsOfDelfi;
    basic_astrodynamics::AccelerationMap accelerationModelMap = createAccelerationModelsMap(
                bodyMap, accelerationMap, bodiesToPropagate, centralBodies );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////             CREATE PROPAGATION SETTINGS            ////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Set Keplerian elements for Delfi.
    Eigen::Vector6d delfiInitialStateInKeplerianElements;
    delfiInitialStateInKeplerianElements( semiMajorAxisIndex ) = 7500.0E3;
    delfiInitialStateInKeplerianElements( eccentricityIndex ) = 0.1;
    delfiInitialStateInKeplerianElements( inclinationIndex ) = unit_conversions::convertDegreesToRadians( 85.3 );

    // **** MODIFY FOR AE4868: set initial state according to your student number
    //    delfiInitialStateInKeplerianElements( argumentOfPeriapsisIndex ) = 0.0;
    //    delfiInitialStateInKeplerianElements( longitudeOfAscendingNodeIndex ) = 0.0;
    //    delfiInitialStateInKeplerianElements( trueAnomalyIndex ) = 0.0;

    // Compute initial Cartesian state
    double earthGravitationalParameter = bodyMap.at( "Earth" )->getGravityFieldModel( )->getGravitationalParameter( );
    const Eigen::Vector6d delfiInitialState = convertKeplerianToCartesianElements(
                delfiInitialStateInKeplerianElements, earthGravitationalParameter );

    // Create propagator settings
    std::shared_ptr< TranslationalStatePropagatorSettings< double > > propagatorSettings;
    propagatorSettings = std::make_shared< TranslationalStatePropagatorSettings< double > >(
                centralBodies, accelerationModelMap, bodiesToPropagate, delfiInitialState, simulationEndEpoch );

    // **** MODIFY FOR AE4868: create integrator settings.
    std::shared_ptr< IntegratorSettings< > > integratorSettings;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////    DEFINE PARAMETERS FOR WHICH SENSITIVITY IS TO BE COMPUTED   ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Define list of parameters to estimate.
    std::vector< std::shared_ptr< EstimatableParameterSettings > > parameterNames;
    // **** MODIFY FOR AE4868, question 2: define list of parameters for which state transition/sensitivity matrix are to be propagated.


    // Create parameters
    std::shared_ptr< estimatable_parameters::EstimatableParameterSet< double > > parametersToEstimate =
            createParametersToEstimate( parameterNames, bodyMap );

    // Print identifiers and indices of parameters to terminal.
    printEstimatableParameterEntries( parametersToEstimate );

    for( unsigned int question = 1; question <= 3; question++ )
    {
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////             PROPAGATE ORBIT AND VARIATIONAL EQUATIONS         /////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // **** MODIFY FOR AE4868, question 1: propagate equations of motion
        // **** MODIFY FOR AE4868, question 2: propagate equations of motion and variational equations


        std::map< double, Eigen::MatrixXd > stateTransitionResult;
        std::map< double, Eigen::MatrixXd > sensitivityResult;
        std::map< double, Eigen::VectorXd > nominalIntegrationResult;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////        PROVIDE OUTPUT TO CONSOLE AND FILES           //////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        std::string outputSubFolder = "DelfiVariationalEquations/";

        // Write perturbed satellite propagation history to file.
        input_output::writeDataMapToTextFile( nominalIntegrationResult,
                                              "singlePerturbedSatellitePropagationHistory.dat",
                                              getOutputPath( ) + outputSubFolder,
                                              "",
                                              std::numeric_limits< double >::digits10,
                                              std::numeric_limits< double >::digits10,
                                              "," );

        input_output::writeDataMapToTextFile( stateTransitionResult,
                                              "singlePerturbedSatelliteStateTransitionHistory.dat",
                                              getOutputPath( ) + outputSubFolder,
                                              "",
                                              std::numeric_limits< double >::digits10,
                                              std::numeric_limits< double >::digits10,
                                              "," );

        input_output::writeDataMapToTextFile( sensitivityResult,
                                              "singlePerturbedSatelliteSensitivityHistory.dat",
                                              getOutputPath( ) + outputSubFolder,
                                              "",
                                              std::numeric_limits< double >::digits10,
                                              std::numeric_limits< double >::digits10,
                                              "," );

        Eigen::Vector6d originalInitialState = delfiInitialState;

        // Iterate over all entries of initial state
        for( unsigned int entry = 0; entry < 6; entry++ )
        {
            // **** MODIFY FOR AE4868, question 3: propagate orbits needed to find limits of validity of linearization.
        }

    }
    // Final statement.
    // The exit code EXIT_SUCCESS indicates that the program was successfully executed.
    return EXIT_SUCCESS;
}

