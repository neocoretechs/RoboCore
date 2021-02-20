package com.neocoretechs.robocore.config;

import java.util.Map;

public class TypedWrapper extends java.util.concurrent.ConcurrentHashMap<String, Object> {
	private static final long serialVersionUID = 1L;
	public TypedWrapper(Object object) {
		this.putAll((Map<? extends String, ? extends Object>) object);
	}
}
