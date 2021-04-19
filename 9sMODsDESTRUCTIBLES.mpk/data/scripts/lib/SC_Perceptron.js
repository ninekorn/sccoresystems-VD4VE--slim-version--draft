//https: //github.com/Brollof/SimplePerceptron by Brollof // The Coding Train youtube tutorial also is the same thing but i stumbled 3 years late on it and found broloff's perceptron project and learned from that one first. 

var weights = [];
var learningRate;

var SC_Perceptron = {

	create_AI_perceptron: function (n, rate)
	{
		learningRate = rate;	
        //weights = new float[n];
        for (var i = 0; i < n; i++)
        {
            weights.push(getSomeRandNum()); // range <-1:1> // 0.000000001 to 999999999
        }
	},
	
    train_perceptron: function(inputs, desired)
    {
        var guess = SC_Perceptron.Guess(inputs);

		//console.PrintError(inputs.length);
        var error = desired - guess;

        for (var i = 0; i < weights.length; i++)
        {
            weights[i] += inputs[i] * learningRate * error;
        }
    },


    SetWeights: function (w) {
        weights = w;
    },

    GetWeights: function()
    {
        return weights;
    },

	Guess: function(inputs)
	{
		var sum = 0;
		for (var i = 0; i < weights.length; i++)
		{
			//console.PrintError(inputs[i]);
			sum += inputs[i] * weights[i];
		}
	
		var result = SC_Perceptron.Activate(sum);

		return result;
	},

	Activate: function (sum)
	{
		if (sum > 0)
			return 1;
		else
			return -1;
	}


    /*Trainer: function(x, y, a)
    {
        //inputs =  [3];

		inputs.push(-1);
        inputs[0] = x;
        inputs[1] = y;
        inputs[2] = 1;
        answer = a;
    }*/
}


function getSomeRandNum()
{
    var num = Math.floor(Math.random() * 999999999) + 1;
	num *= Math.floor(Math.random() * 2) == 1 ? 1 : -1;

	if (num == 0)
    {
        return getSomeRandNum();
    }
	return num * 0.000000001;
}

function Guess(inputs, weights)
{
	var sum = 0;
	for (var i = 0; i < weights.length; i++)
	{
		sum += inputs[i] * weights[i];
	}

	var result = Activate(sum);

	return result;
}

function Activate(sum)
{
    if (sum > 0)
        return 1;
    else
        return -1;
}








	/*Guess: function(float[] inputs)
    {
        float sum = 0;
        for (int i = 0; i < weights.Length; i++)
        {
            sum += inputs[i] * weights[i];
        }
        return this.Activate(sum);
    }

    Activate: function(float sum)
    {
        if (sum > 0)
            return 1;
        else
            return -1;
    }*/












/*
 float[] weights;
    public float learningRate;
    static public System.Random r = new System.Random();

    public Perceptron(int n, float rate)
    {
        this.learningRate = rate;

        this.weights = new float[n];
        // Start with random weights
        for (int i = 0; i < n; i++)
        {
            this.weights[i] = (float)r.NextDouble() * 2 - 1; // range <-1:1>
        }
    }
	*/