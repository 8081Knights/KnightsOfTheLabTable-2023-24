package org.firstinspires.ftc.teamcode.Tests.JSONStuff;

public class JsonObjects {

    public class Movements {
        public class MoveDuo {
            public DaEnum movement;
            public int centimeters;

        }

    }
    public enum DaEnum {
        Foreward,
        Backward,
        MLeft,
        MRight,
        TLeft,
        TRight,
    }
}
