/*    Copyright (c) 2010-2017, Delft University of Technology
 *    All rigths reserved
 *
 *    This file is part of the Tudat. Redistribution and use in source and
 *    binary forms, with or without modification, are permitted exclusively
 *    under the terms of the Modified BSD license. You should have received
 *    a copy of the license with this file. If not, please or visit:
 *    http://tudat.tudelft.nl/LICENSE.
 */

#include <Tudat/JsonInterface/jsonInterface.h>
#include <Tudat/Astrodynamics/BasicAstrodynamics/missionGeometry.h>

#include <boost/lexical_cast.hpp>

using namespace tudat::simulation_setup;
using namespace tudat::propagators;
using namespace tudat::interpolators;
using namespace tudat::json_interface;
using namespace tudat::interpolators;
using namespace tudat;

//! Get path for output directory.
static inline std::string getOutputPath(
        const std::string& extraDirectory = "" )
{
    // Declare file path string assigned to filePath.
    // __FILE__ only gives the absolute path of the header file!
    std::string filePath_( __FILE__ );

    // Strip filename from temporary string and return root-path string.
    std::string reducedPath = filePath_.substr( 0, filePath_.length( ) -
                                                std::string( "delfiPropagationJson.cpp" ).length( ) );
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

//! Function that sets the output files that are used for the current question and run (.../src/DelfiThrust/...)
void setOutputFiles(
        JsonSimulationManager< >& jsonSimulationManager, const int question, const int runIndex )
{
    const std::string outputDirectory = getOutputPath( ) + "DelfiThrust/";
    jsonSimulationManager[ "export" ][ 0 ][ "file" ] =
            outputDirectory + "dependentVariableOutput_" + std::to_string( question ) + "_" +
            std::to_string( runIndex ) + ".dat";
    jsonSimulationManager[ "export" ][ 1 ][ "file" ] =
            outputDirectory + "stateOutput_" + std::to_string( question ) + "_" +
            std::to_string( runIndex ) + ".dat";
}

void setFlightConditions(
        JsonSimulationManager< >& jsonSimulationManager )
{
    if( jsonSimulationManager.getBodyMap( ).at( "delfi" )->getFlightConditions( ) == nullptr )
    {
        jsonSimulationManager.getBodyMap( ).at( "delfi" )->setFlightConditions(
                    createFlightConditions(
                        jsonSimulationManager.getBodyMap( ).at( "delfi" ),
                        jsonSimulationManager.getBodyMap( ).at( "Earth" ),
                        "delfi", "Earth" ) );
    }
}

//! Function to add a thrust acceleration to the propagation settings
/*!
 * \param jsonSimulationManager JSON object containing all simulation settings, read from delfiThrustAnalysis.json. Object is
 * modified by this function
 * \param thrustSettings New thrust settings, which are to be added to the acceleration models.
 */
void setThrustAccelerationSettings(
        JsonSimulationManager< >& jsonSimulationManager,
        const std::shared_ptr< ThrustAccelerationSettings > thrustSettings )
{
    // Retrieve acceleration settings
    std::shared_ptr< TranslationalStatePropagatorSettings< double > > translationalStateSettings =
            std::dynamic_pointer_cast< TranslationalStatePropagatorSettings< double > >(
                jsonSimulationManager.getPropagatorSettings( )->propagatorSettingsMap_.at( translational_state ).at( 0 ) );
    SelectedAccelerationMap accelerationSettingsMap = translationalStateSettings-> getAccelerationSettingsMap( );

    // Add Thrust to acceleration settings
    accelerationSettingsMap[ "delfi" ][ "delfi" ].push_back( thrustSettings );

    // Reset acceleration model settings, and re-process settings
    translationalStateSettings->resetAccelerationModelsMap(
                accelerationSettingsMap, jsonSimulationManager.getBodyMap( ) );
    jsonSimulationManager.getPropagatorSettings( )->resetIntegratedStateModels(
                jsonSimulationManager.getBodyMap( ) );
    jsonSimulationManager.createSimulationObjects( );
}

//! Function to retrieve thrust settings for question 1
/*!
 * \param runIndex Index of the current simulation (e.g. 0, 1 and 2 for thrust case a, b and c)
 */
std::shared_ptr< ThrustAccelerationSettings > getQuestion1ThrustSettings( const int runIndex )
{
    /****** Modify for AE4868: define thrust direction (see ThrustDirectionFromStateGuidanceSettings; question 1 and 2)**********/
    bool thrustAlongVelocityVector; // = ....

    // Define thrust direction
    std::shared_ptr< ThrustDirectionGuidanceSettings > thrustDirectionGuidanceSettings =
            std::make_shared< ThrustDirectionFromStateGuidanceSettings >(
                "Earth", thrustAlongVelocityVector, false );

    /****** Modify for AE4868: define thrust magnitude and specific impulse (question 1 and 2)***********************************/
    double thrustMagnitude;// = ... Thrust in Newtons
    double specificImpulse;// = ... Specific impulse in seconds

    // Define thrust magnitude
    std::shared_ptr< ThrustMagnitudeSettings > thrustMagnitudeSettings =
            std::make_shared< ConstantThrustMagnitudeSettings >(
                thrustMagnitude, specificImpulse );

    // Create and return thrust settings
    return std::make_shared< ThrustAccelerationSettings >(
                thrustDirectionGuidanceSettings, thrustMagnitudeSettings );
}

//! Function to retrieve thrust magnitude for question 3
/*!
 * \param time Current time (included for interface consistency)
 * \param bodyMap List of body objects that defines the environment
 */
double getThrustMagnitudeQuestion3(
        const double time,
        const NamedBodyMap& bodyMap )
{
    bodyMap.at( "delfi" )->getFlightConditions( )->resetCurrentTime( TUDAT_NAN );
    bodyMap.at( "delfi" )->getFlightConditions( )->updateConditions( time );

    /****** Modify for AE4868: define thrust magnitude as a function of altitude (question 3)************************************/
    double thrustMagnitude = 0.0;


    return thrustMagnitude;
}


//! Function to retrieve thrust settings for question 3
/*!
 * \param jsonSimulationManager JSON object containing all simulation settings, read from delfiThrustAnalysis.json.
 */
std::shared_ptr< ThrustAccelerationSettings > getQuestion3ThrustSettings(
        JsonSimulationManager< >& jsonSimulationManager )
{   
    std::shared_ptr< ThrustDirectionGuidanceSettings > thrustDirectionGuidanceSettings;
    std::shared_ptr< ThrustMagnitudeSettings > thrustMagnitudeSettings;

    // Create function for thrust magnitude
    std::function< double( const double ) > thrustMagnitudeFunction =
            std::bind( &getThrustMagnitudeQuestion3, std::placeholders::_1,
                       jsonSimulationManager.getBodyMap( ) );

    // Create function for specific impulse
    double specificImpulse = 300.0;
    std::function< double( const double ) > specificImpulseFunction =
            [ = ]( const double ){ return specificImpulse; };

    // Create thrust magnitude settings
    thrustMagnitudeSettings = std::make_shared< FromFunctionThrustMagnitudeSettings >(
                thrustMagnitudeFunction, specificImpulseFunction );

    /****** Modify for AE4868: define thrust direction (set thrust direction along velocity vector; question 3)******************/
    thrustDirectionGuidanceSettings; = //

    // Create and return thrust settings
    return std::make_shared< ThrustAccelerationSettings >(
                thrustDirectionGuidanceSettings, thrustMagnitudeSettings );
}

//! Function to retrieve thrust settings for question 3
/*!
 * \param runIndex Index of the current simulation (0 and 1 for central difference w.r.t. T; 2 and 3 for central differences
 * w.r.t. I_sp).
 * \param jsonSimulationManager JSON object containing all simulation settings, read from delfiThrustAnalysis.json.
 */
std::shared_ptr< ThrustAccelerationSettings > getQuestion4ThrustSettings(
        const int runIndex )
{
    /****** Modify for AE4868: set thrust direction (question 4)*****************************************************************/
    std::shared_ptr< ThrustDirectionGuidanceSettings > thrustDirectionGuidanceSettings;
    thrustDirectionGuidanceSettings; // = ...

    /****** Modify for AE4868: set thrust magnitude and specific impulse for central difference (question 4)*********************/
    std::shared_ptr< ThrustMagnitudeSettings > thrustMagnitudeSettings;
    thrustMagnitudeSettings; // = ...

    return std::make_shared< ThrustAccelerationSettings >(
                thrustDirectionGuidanceSettings, thrustMagnitudeSettings );
}

//! Function to get thrust direction in orbital-plane, perpendicular to velocity vector
/*!
 * \param time Current time (included for interface consistency)
 * \param bodyMap List of body objects that defines the environment
 */
Eigen::Vector3d getNormalThrustDirection(
        const double time,
        const NamedBodyMap& bodyMap  )
{
    // Compute state of Delfi w.r.t. Earth (position and velocity)
    Eigen::Vector6d vehicleRelativeState = bodyMap.at( "delfi" )->getState( ) -
            bodyMap.at( "Earth" )->getState( );

    /****** Modify for AE4868: define thrust direction (question 5)**************************************************************/
    Eigen::Vector3d thrustDirection; // = ...
    return thrustDirection;

}

//! Function to get thrust direction perpendicular to orbital-plane
/*!
 * \param time Current time (included for interface consistency)
 * \param bodyMap List of body objects that defines the environment
 */
Eigen::Vector3d getCrossTrackThrustDirection(
        const double time,
        const NamedBodyMap& bodyMap  )
{
    // Compute state of Delfi w.r.t. Earth (position and velocity)
    Eigen::Vector6d vehicleRelativeState = bodyMap.at( "delfi" )->getState( ) -
            bodyMap.at( "Earth" )->getState( );

    /****** Modify for AE4868: define thrust direction (question 5)**************************************************************/
    Eigen::Vector3d thrustDirection; // = ...
    return thrustDirection;
}

//! Function to retrieve thrust settings for question 5
/*!
 * \param runIndex Index of the current simulation.
 * \param jsonSimulationManager JSON object containing all simulation settings, read from delfiThrustAnalysis.json.
 */
std::shared_ptr< ThrustAccelerationSettings > getQuestion5ThrustSettings(
        const int runIndex, JsonSimulationManager< >& jsonSimulationManager )
{
    std::shared_ptr< ThrustDirectionGuidanceSettings > thrustDirectionGuidanceSettings;
    std::shared_ptr< ThrustMagnitudeSettings > thrustMagnitudeSettings;

    // Create function for thrust direction (in orbital plane; perpendicular to velocity)
    std::function< Eigen::Vector3d( const double ) > normalThrustDirection =
            std::bind( &getNormalThrustDirection, std::placeholders::_1, jsonSimulationManager.getBodyMap( ) );

    // Create function for thrust direction (perpendicular to orbital plane)
    std::function< Eigen::Vector3d( const double ) > crossTrackThrustDirection =
            std::bind( &getCrossTrackThrustDirection, std::placeholders::_1, jsonSimulationManager.getBodyMap( ) );

    /****** Modify for AE4868: set thrust magnitude and direction (question 5) **************************************************/

    return std::make_shared< ThrustAccelerationSettings >(
                thrustDirectionGuidanceSettings, thrustMagnitudeSettings );
}

//! Execute propagation of orbits of Apollo during entry using the JSON Interface.
int main( )
{
    // Retrieve current directory
    const std::string cppFilePath( __FILE__ );
    const std::string cppFolder = cppFilePath.substr( 0, cppFilePath.find_last_of("/\\") + 1 );

    // Load JSON file
    JsonSimulationManager< > jsonSimulationManager( cppFolder + "delfiThrustAnalysis.json" );

    // *********************** ITERATE OVER QUESTIONS ************************
    for( unsigned int question = 3; question <= 5; question++ )
    {
        unsigned int numberOfCases = 0;
        /****** Modify for AE4868 (define number of runs per question; question 1-5) ********************************************/

        // Iterate over all cases for current question
        for ( unsigned int runIndex = 0; runIndex < numberOfCases; ++runIndex )
        {
            // Output to notify start of current run
            std::cout << "Running case " << runIndex  << " for question " <<question<< std::endl;

            // Reset JSON data to original configuration
            jsonSimulationManager.resetJsonObject( jsonSimulationManager.getOriginalJsonObject( ) );

            // Define output files
            setOutputFiles(  jsonSimulationManager, question, runIndex );

            /****** Modify for AE4868: change initial Kepler elements (questions 2 and 3) ***************************************/

            // Parse JSON data
            jsonSimulationManager.updateSettings( );

            // Retrieve thrust settings for specific question
            if( question == 1 || question == 2 )
            {
                if( runIndex > 0 )
                {
                    setThrustAccelerationSettings(
                                jsonSimulationManager, getQuestion1ThrustSettings( runIndex ) );
                }
            }
            else if( question == 3 )
            {
                setFlightConditions( jsonSimulationManager );
                setThrustAccelerationSettings(
                            jsonSimulationManager, getQuestion3ThrustSettings( jsonSimulationManager ) );
            }
            else if( question == 4 )
            {
                setThrustAccelerationSettings(
                            jsonSimulationManager, getQuestion4ThrustSettings( runIndex ) );
            }
            else if( question == 5 )
            {
                setThrustAccelerationSettings(
                            jsonSimulationManager, getQuestion5ThrustSettings( runIndex, jsonSimulationManager ) );
            }

            // Propagate dynamics
            jsonSimulationManager.runPropagation( );

            // Save data to files
            jsonSimulationManager.exportResults( );

            // Suppress JSON warnings
            if ( runIndex == 0 )
            {
                jsonSimulationManager[ "options" ][ "unusedKey" ] = tudat::json_interface::continueSilently;
            }
        }
    }

    return EXIT_SUCCESS;
}
