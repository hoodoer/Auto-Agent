// This neural network class is 
// from AI for Game Developers, 
// Oreilly Publishing. It has 
// minor modifications by 
// Drew Kirkpatrick, drew.kirkpatrick@gmail.com
// allowing the class to re-read the weights of 
// a trained brain from a file. 

#include <iostream>
using namespace std;
#include <string>


#ifndef NEURALNET_H
#define NEURALNET_H

class NeuralNetworkLayer
{
public:
	int			NumberOfNodes;
	int			NumberOfChildNodes;
	int			NumberOfParentNodes;
	double**	Weights;
	double**	WeightChanges;
	double*		NeuronValues;
	double*		DesiredValues;
	double*		Errors;
	double*		BiasWeights;
	double*		BiasValues;
	double		LearningRate;

	bool		LinearOutput;
	bool		UseMomentum;
	double		MomentumFactor;

	NeuralNetworkLayer*		ParentLayer;
	NeuralNetworkLayer*		ChildLayer;

	NeuralNetworkLayer();

	void	Initialize(int	NumNodes, NeuralNetworkLayer* parent, NeuralNetworkLayer* child);
	void	CleanUp(void);
	void	RandomizeWeights(void);
	void	CalculateErrors(void);
	void	AdjustWeights(void);	
	void	CalculateNeuronValues(void);
	
};

// Implements a 3-Layer neural network with one input layer, one hidden layer, and one output layer
class NeuralNetwork 
{
public:
	NeuralNetworkLayer	InputLayer;
	NeuralNetworkLayer	HiddenLayer;
	NeuralNetworkLayer	OutputLayer;

	void	Initialize(int nNodesInput, int nNodesHidden, int nNodesOutput);
	void	CleanUp();
	void	SetInput(int i, double value);
	double	GetOutput(int i);
	void	SetDesiredOutput(int i, double value);
	void	FeedForward(void);
	void	BackPropagate(void);
	int		GetMaxOutputID(void);
	double	CalculateError(void);
	void	SetLearningRate(double rate);
	void	SetLinearOutput(bool useLinear);
	void	SetMomentum(bool useMomentum, double factor);
	void	DumpData(string filename);
	void    ReadData(string filename);
};

#endif   // NEURALNET_H
