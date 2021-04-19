using(console);

var IMCache =
    {
        //-----------------------------------
        // public

        //clears current cache
        Clear: function()
        {
            this.cached = {};
        },
        AddValue: function(name, val)
        {
            if (!this.cached.hasOwnProperty(name))
            {
                this.cached[name] = val;
            }
            else
            {
                this.cached[name] += val;
            }
        },
		CheckValue: function(name)
        {
			//console.PrintError("someTesting " + this.cached[name]);

			
			//this.cached[name]
			//this.cached[name] = "hull_avalanche";

			//console.PrintError("some testing");



            /*if (this.cached.hasOwnProperty(name))
            {
                //this.cached[name] = val;
				//console.PrintError("value exists");
            }*/
        },


        MulValue: function(name, val)
        {
            this.cached[name] *= val;
        },
        GetValue: function(name)
        {
            return this.cached[name];
        },
        ApplyCacheFunction: function(ship_id, func)
        {
            for (var key in this.cached)
            {
                func(ship_id, key, this.cached[key]);
            }
        },
        //--------------------------------
        // private

        cached: {}
    };

console.Print("-- IMCache lib initialized");