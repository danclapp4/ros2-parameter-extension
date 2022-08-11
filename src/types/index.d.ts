declare module "parameter_types" {

    export type Parameter = {
        name: string;
        value: ParameterValue;
    }

    export type ParameterValue = {
        type: number;
        bool_value: boolean;
        integer_value: number;
        double_value: number;
        string_value: string;
        byte_array_value: number[];
        bool_array_value: boolean[];
        integer_array_value: number[];
        double_array_value: number[];
        string_array_value: string[];
    }

    export type SetSrvParam = { 
        name?: string;
        value?: {
            type: number;
            bool_value?: boolean;
            integer_value?: number;
            double_value?: number;
            string_value?: string;
            byte_array_value?: number[];
            bool_array_value?: boolean[];
            integer_array_value?: number[];
            double_array_value?: number[];
            string_array_value?: string[];
        }
    }

}